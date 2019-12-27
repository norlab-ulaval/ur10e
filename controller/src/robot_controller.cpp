#include "robot_controller.h"



//----------------------------
// object management
//----------------------------
RobotController::RobotController()
{
    robot_model.calibrate();
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
RobotController::~RobotController()
{
    delete traj_client;
}



//----------------------------
// State machine functions
//----------------------------
void RobotController::control(const ros::TimerEvent&)
{
    // if (robot_mode == -100)
    // {
    //     print("Warning, the mode of the robot is still unknown.");
    // }

    if (events.has_error())
    {
        state = State::error;
    }

    switch(state)
    {
    case State::uninitialized:
    {
        if (events == Command::init)
            state = State::initializing;
        break;
    }


    case State::initializing:
    {
        init_control();
        if (events == Command::done)
        {
            ROS_INFO("Done.");
            state = State::standstill;
        }
        break;
    }


    case State::standstill:
    {
        if (events == Command::start_cart_velocity)
        {
            robot_model.fk(cur_joint_pos, cur_cart_pos_goal, cur_cart_rot_goal);
            state = State::cart_velocity;
            ROS_INFO("Joystick...");
        }
        else if (events == Command::start_joint_positiom)
        {
            state = State::joint_position;
            ROS_INFO("Joint position...");
        }
        else if (events == Command::start_home)
        {
            Vec6 target = {0, -pi_2, pi_2, 0, 0, 0};
            Vec6 max_vel = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
            replace_target(target, max_vel);
            state = State::homing;
            ROS_INFO("Homing...");
        }
        break;
    }


    case State::homing:
    {
        position_control();
        if (events == Command::stop)
        {
            state = State::stopping;
            ROS_INFO("Stopping...");
        }
        else if (events == Command::done)
        {
            state = State::standstill;
            ROS_INFO("Done.");
        }
        break;
    }


    case State::joint_position:
    {
        position_control();
        if (events == Command::stop)
        {
            state = State::stopping;
            ROS_INFO("Stopping...");
        }
        break;
    }


    case State::cart_velocity:
    {
        velocity_control();
        if (events == Command::stop)
        {
            state = State::stopping;
            ROS_INFO("Stopping...");
        }
        break;
    }


    case State::stopping:
    {
        traj_client->cancelAllGoals();
        if (small(joints_msg->actual.velocities))
        {
            state = State::standstill;
            ROS_INFO("Done.");
        }
        break;
    }


    case State::error:
    {
        traj_client->cancelAllGoals();
        if (events == Command::reset)
        {
            events.clear_errors();
            state = State::uninitialized;
            ROS_INFO("Errors reset, returning to uninitialized...");
        }
        break;
    }


    default:
    {
        state = State::error;
        events.add(Error::unknown_state);
        ROS_ERROR("The internal state changed to an unknown state. State %d is not handled.", state);
    }
    } // switch (state)



    events.clear();

    state_msg.state = state;
    state_pub.publish(state_msg);
}
void RobotController::init_control()
{
    enum class ST
    {
        power,
        brakes,
        load,
        play,
    };
    using std_srvs::Trigger;
    using ur_dashboard_msgs::Load;
    using ur_dashboard_msgs::RobotMode;
    const auto call_trg  = ros::service::call<Trigger>;
    const auto call_load = ros::service::call<Load>;
    const auto call_state = ros::service::call<GetProgramState>;


    static ST st = ST::power;
    switch (st)
    {
    // power up the robot
    case ST::power:
    {
        if (robot_mode < RobotMode::POWER_OFF)
        {
            events.add(Error::robot_wrong_mode);
            ROS_ERROR("The robot should be ready but it's not. RobotMode: %d", robot_mode);
            st = ST::power;
        }
        else if (robot_mode >= RobotMode::IDLE)
        {
            st = ST::brakes;
        }
        else
        {
            Trigger srv;
            call_trg("/ur_hardware_interface/dashboard/power_on", srv);
            ROS_INFO("Powering up...");
        }
        break;
    }

    // release the brakes
    case ST::brakes:
    {
        if (robot_mode < RobotMode::IDLE)
        {
            events.add(Error::robot_wrong_mode);
            ROS_ERROR("The robot should be powered on, but it's not. Mode: %d", robot_mode);
            st = ST::power;
        }
        else if (robot_mode >= RobotMode::RUNNING)
        {
            st = ST::load;
        }
        else
        {
            Trigger srv;
            call_trg("/ur_hardware_interface/dashboard/brake_release", srv);
            ROS_INFO("Releasing brakes...");
        }
        break;
    }
    case ST::load:
    {
        if (robot_mode != RobotMode::RUNNING)
        {
            events.add(Error::robot_wrong_mode);
            ROS_ERROR("The robot is not in the correct mode to load the program. RobotMode: %d", robot_mode);
            st = ST::power;
        }
        else
        {
            Load srv;
            srv.request.filename = "external.urp";
            call_load("ur_hardware_interface/dashboard/load_program", srv);
            ROS_INFO("Loading external.urp program...");
            if (srv.response.success)
            {
                st = ST::play;
            }
            else
            {
                events.add(Error::program_load);
                ROS_ERROR("Error loading program. Reply: %s.", srv.response.answer.c_str());
                st = ST::power;
            }
        }
        break;
    }
    case ST::play:
    {
        if (robot_mode < RobotMode::RUNNING && robot_mode != RobotMode::BACKDRIVE)
        {
            events.add(Error::robot_wrong_mode);
            ROS_ERROR("The robot is not in the correct mode to play the program. RobotMode: %d", robot_mode);
            st = ST::power;
        }
        else
        {
            // get the program state
            GetProgramState state_srv;
            call_state("ur_hardware_interface/dashboard/program_state", state_srv);
            ROS_INFO("Playing external.urp program...");
            if (state_srv.response.state.state == state_srv.response.state.PLAYING
                && state_srv.response.program_name == "external.urp")
            {
                events.add(Command::done);
            }
            else
            {
                Trigger srv;
                call_trg("ur_hardware_interface/dashboard/play", srv);
                if (srv.response.success)
                {
                    events.add(Command::done);
                }
                else
                {
                    events.add(Error::program_play);
                    ROS_ERROR("Error playing program. Reply: %s.", srv.response.message.c_str());
                }
            }
            st = ST::power;
        }
        break;
    }
    }
}
void RobotController::velocity_control()
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
    if (robot_model.ik(next_pos, next_rot, cur_joint_pos, joint_goal))
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
void RobotController::position_control()
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
void RobotController::replace_target(Vec6& q, Vec6& v)
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
void RobotController::add_target(Vec6& q, double T)
{
    validate_and_send(q, T);
}



//----------------------------
// utility functions
//----------------------------
bool RobotController::validate_and_send(Vec6& joints, double dt)
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
bool RobotController::validate_and_send(Vec6& joints)
{
    return validate_and_send(joints, delta);
}



//----------------------------
// service callbacks
//----------------------------
bool RobotController::init(Empty::Request&, Empty::Response&)
{
    events.add(Command::init);
    return true;
}
bool RobotController::stop_trajectory(Empty::Request&, Empty::Response&)
{
    events.add(Command::stop);
    return true;
}
bool RobotController::start_velocity(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_cart_velocity);
    return true;
}
bool RobotController::start_position(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_joint_positiom);
    return true;
}
bool RobotController::start_home(Empty::Request&, Empty::Response&)
{
    events.add(Command::start_home);
    return true;
}
bool RobotController::reset_errors(Empty::Request&, Empty::Response&)
{
    events.add(Command::reset);
    return true;
}


//----------------------------
// subscriber callbacks
//----------------------------
void RobotController::robot_mode_callback(const RobotMode::ConstPtr& msg)
{
    robot_mode = msg->mode;
}
void RobotController::state_callback(const TrajState::ConstPtr& msg)
{
    joints_msg = msg;
    cur_joint_pos = msg->actual.positions;
}
void RobotController::joy_callback(const Joy::ConstPtr& msg)
{
    //-- reset to zero
    cart_vel_goal = {0, 0, 0, 0, 0, 0};
    gripper_command = 0;

    //-- compute linear velocity
    cart_vel_goal.x = -msg->axes[0] * linear_speed;
    cart_vel_goal.y = msg->axes[1] * linear_speed;
    if (msg->buttons[6] && !msg->buttons[7])
    {
        cart_vel_goal.z = linear_speed;
    }
    else if (msg->buttons[7] && !msg->buttons[6])
    {
        cart_vel_goal.z = -linear_speed;
    }

    //-- compute angular velocity
    // x axis
    if (msg->axes[3] > 0.2)
    {
        cart_vel_goal.wx = -angular_speed;
    }
    else if (msg->axes[3] < -0.2)
    {
        cart_vel_goal.wx = angular_speed;
    }
    // y axis
    if (msg->axes[2] > 0.2)
    {
        cart_vel_goal.wy = angular_speed;
    }
    else if (msg->axes[2] < -0.2)
    {
        cart_vel_goal.wy = -angular_speed;
    }
    // z axis
    if (msg->buttons[5] && !msg->buttons[4])
    {
        cart_vel_goal.wz = angular_speed;
    }
    else if (msg->buttons[4] && !msg->buttons[5])
    {
        cart_vel_goal.wz = -angular_speed;
    }

    //-- button to start the velocity control
    if (msg->buttons[2])
    {
        events.add(Command::stop);
    }
    else if (msg->buttons[1])
    {
        events.add(Command::start_cart_velocity);
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
