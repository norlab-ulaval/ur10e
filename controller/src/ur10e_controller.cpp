#include "ur10e_controller.h"



//----------------------------
// object management
//----------------------------
Ur10eController::Ur10eController()
{
    robot.calibrate();
    state = State::uninitialized;
    traj_client = new TrajClient("scaled_pos_traj_controller/follow_joint_trajectory", false);
    traj_goal.trajectory.points.resize(1);
    traj_goal.trajectory.joint_names.resize(6);
    traj_goal.trajectory.joint_names[0] = "shoulder_pan_joint";
    traj_goal.trajectory.joint_names[1] = "shoulder_lift_joint";
    traj_goal.trajectory.joint_names[2] = "elbow_joint";
    traj_goal.trajectory.joint_names[3] = "wrist_1_joint";
    traj_goal.trajectory.joint_names[4] = "wrist_2_joint";
    traj_goal.trajectory.joint_names[5] = "wrist_3_joint";
}
Ur10eController::~Ur10eController()
{
    delete traj_client;
}



//----------------------------
// State machine functions
//----------------------------
void Ur10eController::control(const ros::TimerEvent&)
{
    assert_ur(
        robot_mode >= RobotMode::BACKDRIVE
        || state == State::uninitialized
        || state == State::error,
        "The robot is not running, but the controller state expects it to be."
    );

    if (events.has_error())
    {
        state = State::error;
        next_state = State::error;
    }

    switch(state)
    {

    case State::uninitialized:
    {
        if (events == Command::init)
        {
            next_state = State::standstill;
        }

        break;
    }


    case State::standstill:
    {
        if (events == Command::start_cart_velocity)
        {
            next_state = State::cart_velocity;
        }
        else if (events == Command::start_joint_positiom)
        {
            next_state = State::joint_position;
        }
        else if (events == Command::start_home)
        {
            next_state = State::homing;
        }
        break;
    }


    case State::homing:
    {
        assert_ur(robot_mode != RobotMode::BACKDRIVE, "Backdrive mode is not allowed while moving.");

        if (state != last_state)
        {
            Vec6 target = {0, -pi_2, -pi_2, -pi_2, pi_2, 0};
            Vec6 max_vel = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
            replace_target(target, max_vel);
            ROS_INFO("Homing...");
        }

        position_control();

        if (events == Command::stop)
        {
            next_state = State::stopping;
        }
        else if (events == Command::done)
        {
            ROS_INFO("Done.");
            next_state = State::standstill;
        }
        break;
    }


    case State::joint_position:
    {
        assert_ur(robot_mode != RobotMode::BACKDRIVE, "Backdrive mode is not allowed while moving.");

        if (state != last_state)
        {
            ROS_INFO("Joint position mode.");
        }

        position_control();

        if (events == Command::stop)
        {
            next_state = State::stopping;
        }
        break;
    }


    case State::cart_velocity:
    {
        if(state != last_state)
        {
            robot.fk(cur_joint_pos, cur_cart_pos_goal, cur_cart_rot_goal);
            ROS_INFO("Joystick mode.");
        }

        velocity_control();

        if (events == Command::stop)
        {
            next_state = State::stopping;
        }
        break;
    }


    case State::stopping:
    {
        assert_ur(robot_mode != RobotMode::BACKDRIVE, "Backdrive mode is not allowed while moving.");

        if (state != last_state)
        {
            ROS_INFO("Stopping...");
            traj_client->cancelAllGoals();
        }

        if (small(joints_msg->actual.velocities))
        {
            ROS_INFO("Done.");
            next_state = State::standstill;
        }
        break;
    }


    case State::error:
    {
        if (state != last_state)
        {
            traj_client->cancelAllGoals();
        }

        if (events == Command::reset_errors)
        {
            events.clear_errors();
            next_state = State::uninitialized;
            ROS_INFO("Errors reset, returning to uninitialized.");
        }
        break;
    }


    default:
    {
        next_state = State::error;
        events.add(Error::unknown_state);
        ROS_ERROR("The controller state changed to an unknown state. State %d is not handled.", state);
    }
    } // switch (state)



    events.clear();

    last_state = state;
    state = next_state;

    state_msg.state = state;
    state_pub.publish(state_msg);
}
void Ur10eController::velocity_control()
{
    // get current cartesian position and orientation

    // update last position and rotation in cartesian space
    // todo: validate with the fk?
    Vec3 next_pos = cur_cart_pos_goal;
    if (norm_squared(cart_vel_goal.first3()) > 1e-6)
    {
        next_pos += cart_vel_goal.first3() * delta;
    }
    Mat3 next_rot = cur_cart_rot_goal;
    if (norm_squared(cart_vel_goal.last3()) > 1e-6)
    {
        // note: premultiplied by next_rot to rotate about the tool axes
        Vec3 u = next_rot*cart_vel_goal.last3();
        double th = norm(u);
        u /= th;
        th *= delta;

        double c = cos(th);
        double s = sin(th);
        double cm1 = 1.0-c;

        Mat3 delta_rot =
        {
            // https://fr.wikipedia.org/wiki/Matrice_de_rotation#Axe_de_rotation
            u.x*u.x*cm1 + c,
            u.x*u.y*cm1 + u.z*s,
            u.x*u.z*cm1 - u.y*s,

            u.x*u.y*cm1 - u.z*s,
            u.y*u.y*cm1 + c,
            u.y*u.z*cm1 + u.x*s,

            u.x*u.z*cm1 + u.y*s,
            u.y*u.z*cm1 - u.x*s,
            u.z*u.z*cm1 + c
        };
        // note: delta rotation is in TOOL coordinates

        next_rot = delta_rot * next_rot;
        if (!next_rot.is_unit())
            ROS_WARN("Rotation matrix is no longer orthogonal!");
    }

    // compute the associated joint positions (inverse kinematics)
    Vec6 joint_goal = cur_joint_pos;
    if (robot.ik(next_pos, next_rot, cur_joint_pos, joint_goal))
    {
        cur_cart_pos_goal = next_pos;
        cur_cart_rot_goal = next_rot;
    }
    else
    {
        joint_goal = cur_joint_pos;
        ROS_WARN("Inverse kinematics did not converge.");
    }

    // send message
    validate_and_send(joint_goal);
}
void Ur10eController::position_control()
{
    auto s = traj_client->getState();
    if (s == s.LOST)
    {
        // do nothing
    }
    else if (s == s.ACTIVE || s == s.PENDING)
    {
        // do nothing
    }
    else if (s == s.PREEMPTED || s == s.ABORTED || s == s.RECALLED || s == s.REJECTED)
    {
        events.add(Error::traj_action_err);
        auto err = traj_client->getResult();
        ROS_ERROR("The trajectory action encountered an error: %s.", err->error_string.c_str());
    }
    else if (s == s.SUCCEEDED)
    {
        events.add(Command::done);
    }
}



//----------------------------
// Action server functions
//----------------------------
void Ur10eController::replace_target(Vec6& q, Vec6& v)
{
    auto s = traj_client->getState();
    if (s == s.ACTIVE || s==s.PENDING)
    {
        traj_client->cancelGoal();
    }

    Vec6 disp = q - cur_joint_pos;
    double time = 0.0;

    // note: this presumes a constant velocity and negligeable accel time!!!
    for (uint i = 0; i < 6; i++)
    {
        double T = 0.0;
        T = disp[i] / v[i];
        time = T>time ? T : time;
    }

    if (time < 0)
    {
        events.add(Error::target_computation_err);
        ROS_ERROR("Error in homing computation, the computed time is negative. Time: %f", time);
    }
    else
    {
        validate_and_send(q, time);
    }
}
void Ur10eController::add_target(Vec6& q, double T)
{
    validate_and_send(q, T);
}



//----------------------------
// utility functions
//----------------------------
bool Ur10eController::validate_and_send(Vec6& joints, double dt)
{
    bool success = true;
    // todo: add tests on the joint position

    // check that the length of the trajectory is at least the minimum delta time.
    dt = dt < delta ? delta : dt;

    // note: for now, just forward the target to the robot.
    traj_goal.trajectory.points[0].positions.assign(joints.begin(), joints.end());
    traj_goal.trajectory.header.stamp = ros::Time::now();
    traj_goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    traj_client->sendGoal(traj_goal);

    return success;
}
bool Ur10eController::validate_and_send(Vec6& joints)
{
    return validate_and_send(joints, delta);
}



//----------------------------
// service callbacks
//----------------------------
bool Ur10eController::init(Empty::Request&, Empty::Response&)
{
    events.add(Command::init);
    return true;
}
bool Ur10eController::stop_trajectory(Empty::Request&, Empty::Response&)
{
    events.add(Command::stop);
    return true;
}
bool Ur10eController::start_velocity(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_cart_velocity);
    return true;
}
bool Ur10eController::start_position(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_joint_positiom);
    return true;
}
bool Ur10eController::start_home(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_home);
    return true;
}
bool Ur10eController::reset_errors(Empty::Request&, Empty::Response&)
{
    events.add(Command::reset_errors);
    return true;
}



//----------------------------
// subscriber callbacks
//----------------------------
void Ur10eController::robot_mode_callback(const RobotMode::ConstPtr& msg)
{
    robot_mode = msg->mode;
}
void Ur10eController::state_callback(const TrajState::ConstPtr& msg)
{
    joints_msg = msg;
    cur_joint_pos = msg->actual.positions;
}
void Ur10eController::joy_callback(const Joy::ConstPtr& msg)
{
    //-- reset to zero
    cart_vel_goal = {0, 0, 0, 0, 0, 0};
    gripper_command = 0;

    //-- compute linear velocity
    cart_vel_goal.x = -msg->axes[0] * linear_speed * speed_scale;
    cart_vel_goal.y = msg->axes[1] * linear_speed * speed_scale;
    if (msg->buttons[6] && !msg->buttons[7])
    {
        cart_vel_goal.z = linear_speed * speed_scale;
    }
    else if (msg->buttons[7] && !msg->buttons[6])
    {
        cart_vel_goal.z = -linear_speed * speed_scale;
    }

    //-- compute angular velocity
    // x axis
    if (msg->axes[3] > 0.2)
    {
        cart_vel_goal.wx = -angular_speed * speed_scale;
    }
    else if (msg->axes[3] < -0.2)
    {
        cart_vel_goal.wx = angular_speed * speed_scale;
    }
    // y axis
    if (msg->axes[2] > 0.2)
    {
        cart_vel_goal.wy = angular_speed * speed_scale;
    }
    else if (msg->axes[2] < -0.2)
    {
        cart_vel_goal.wy = -angular_speed * speed_scale;
    }
    // z axis
    if (msg->buttons[5] && !msg->buttons[4])
    {
        cart_vel_goal.wz = angular_speed * speed_scale;
    }
    else if (msg->buttons[4] && !msg->buttons[5])
    {
        cart_vel_goal.wz = -angular_speed * speed_scale;
    }

    //-- button to start the velocity control
    if (msg->buttons[2])
    {
        events.add(Command::stop);
    }
    if (msg->buttons[0] && !msg->buttons[3])
    {
        gripper_command = 1;
    }
    else if (msg->buttons[3] && !msg->buttons[0])
    {
        gripper_command = -1;
    }

    //-- store message
    joy_msg = msg;
}
void Ur10eController::speed_scale_callback(const std_msgs::Float64::ConstPtr& msg)
{
    speed_scale = msg->data;
}



//----------------------------
// Utility functions
//----------------------------
void print(const char* prefix, const Vec3& v)
{
    ROS_INFO("%s%f, %f, %f", prefix, v.x, v.y, v.z);
}
void print(const char* prefix, const Vec6& v)
{
    ROS_INFO("%s%f, %f, %f, %f, %f, %f", prefix, v.j1, v.j2, v.j3, v.j4, v.j5, v.j6);
}
void print(const char* prefix, const Vec12& v)
{
    ROS_INFO("%s%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", prefix,
        v.data[0], v.data[1], v.data[2],
        v.data[3], v.data[4], v.data[5],
        v.data[6], v.data[7], v.data[8],
        v.data[9], v.data[10], v.data[11]);
}
void print(const char* prefix, const Mat3& m)
{
    ROS_INFO("%s%f, %f, %f, %f, %f, %f, %f, %f, %f", prefix,
        m.data[0], m.data[1], m.data[2],
        m.data[3], m.data[4], m.data[5],
        m.data[6], m.data[7], m.data[8]);
}
void print(const char* prefix, const uint& state)
{
    ROS_INFO("%s%u", prefix, state);
}
void print(const char* prefix, const Command& cmd)
{
    ROS_INFO("%s%d", prefix, (int)cmd);
}
Vec6 to_deg(const Vec6& q)
{
    return q * 57.2957795131;
}
