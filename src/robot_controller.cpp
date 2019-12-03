#include "robot_controller.h"


/** Constructor
 * - Contruction of the object. Launch the calibration
 * of the robot model.
 *
 */
RobotController::RobotController()
{
    robot_model.calibrate();
}



/**
 * - Trajectory state callback, called when trajectory state message is
 *      received.
 * - Read the state of the robot and update the object's variables such as the
 *      last known robot time and position.
 *
 * (intput) msg -> pointer to the current state of the trajectory following action.
 */
void RobotController::state_callback(const MsgTrajState::ConstPtr& msg)
{
    static bool first = true;
    if (first)
    {
        first = false;
        command_msg.goal.trajectory.joint_names = msg->joint_names;
        command_msg.goal.trajectory.points.resize(1);
    }

    joints_msg = msg;
    cur_joint_pos = msg->actual.positions;

    control();
}



/**
 * - Joy message callback. Called every tick of the joystick node.
 * - Read the joystick state and update the velocity command. This is used by
 *      the cartesian velocity control mode.
 *
 * (intput) joy -> pointer containing the current joy state
 */
void RobotController::joy_callback(const MsgJoy::ConstPtr& msg)
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
        command = Command::stop;
    }
    else if (msg->buttons[1])
    {
        command = Command::start_cart_velocity;
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



/**
 * - Main control function, called by state_callback. This function
 * generates the next point for the manipulator to follow and sends them on
 * topics.
 * - This is where the state machine resides and therefore the different control
 * modes functions are called from here.
 *
 */
void RobotController::control()
{
    switch(state)
    {
    case State::standstill:
    {
        if (command == Command::start_cart_velocity)
        {
            robot_model.fk(cur_joint_pos, cur_cart_pos_goal, cur_cart_rot_goal);
            state = State::cart_velocity;
        }
        break;
    }


    // Control the robot with the joystick in cartesian velocity mode
    case State::cart_velocity:
    {
        if (command == Command::stop)
        {
            state = State::stopping;
            break; // --> don't execute
        }

        velocity_control();
        break;
    }


    // safely stop the robot and return to standstill when done
    case State::stopping:
    {
        bool done = stopping_control();

        if (done)
        {
            state = State::standstill;
        }
        break;
    }


    // Error handling
    // Cannot do anything unless the reset command is set and there are no more errors
    case State::error:
    {
        if (command == Command::reset)
        {
            state = State::standstill;
        }
        break;
    }
    }

    command = Command::none; // reset to not reprocess the same command twice
}



/**
 * - Control function for the velocity control.
 * - Uses the current joystick command to determine the velocity command.
 * - Transforms the velocity command into a a new position command (cartesian).
 * - Performs the inverse kinematics of that point to obtain a goal joint pose.
 * - Sends the position points to the robot.
 *
 */
void RobotController::velocity_control()
{
    // get current cartesian position and orientation
    const double duration = 0.1;

    // update last position and rotation in cartesian space
    // todo: validate with the fk?
    Vec3 next_pos = cur_cart_pos_goal;
    if (norm_squared(cart_vel_goal.first3()) > 1e-6)
    {
        next_pos += cart_vel_goal.first3() * duration;
        cur_cart_pos_goal = next_pos;
    }
    Mat3 next_rot = cur_cart_rot_goal;
    if (norm_squared(cart_vel_goal.last3()) > 1e-6)
    {
        // note: premultiplied by next_rot to rotate about the tool axes
        Vec3 u = next_rot*cart_vel_goal.last3();
        double th = norm(u);
        if (small(th))
            ROS_WARN("normalizing a vector that is too close to zero, norm: %f", th);
        u /= th;

        if (!is_unit(u))
            ROS_WARN("The unit direction vector is not really unit, norm: %f", norm(u));

        th *= duration;
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

        cur_cart_rot_goal = next_rot;
    }

    // compute the associated joint positions (inverse kinematics)
    Vec6 joint_goal = robot_model.ik(next_pos, next_rot, cur_joint_pos);

    // send goal
    command_msg.goal.trajectory.header.stamp = joints_msg->header.stamp;
    command_msg.goal.trajectory.points[0].positions = {joint_goal.begin(), joint_goal.end()};
    command_msg.goal.trajectory.points[0].time_from_start = ros::Duration(duration);
    command_pub.publish(command_msg);
}



bool RobotController::stopping_control()
{
    // todo: properly cancel the current trajectory
    Vec6 vel = joints_msg->actual.velocities;
    return small(vel);
}







/** Utility functions
*/
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
void print(const char* prefix, const State& state)
{
    ROS_INFO("%s%d", prefix, (int)state);
}
void print(const char* prefix, const Command& cmd)
{
    ROS_INFO("%s%d", prefix, (int)cmd);
}
Vec6 to_deg(const Vec6& q)
{
    return q * 57.2957795131;
}
