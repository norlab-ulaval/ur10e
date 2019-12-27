#pragma once

#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <ur_dashboard_msgs/RobotMode.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ur10e_messages/State.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/GetProgramState.h>

#include "robot_model.h"
#include "robot_events.h"
#include "linear.h"

const double angular_speed = 1.0; // rad/s
const double linear_speed = 0.2;  // m/s

using TrajGoal = control_msgs::FollowJointTrajectoryGoal;
using TrajState = control_msgs::JointTrajectoryControllerState;
using sensor_msgs::Joy;
using ur_dashboard_msgs::RobotMode;
using ur_dashboard_msgs::GetProgramState;
using std_srvs::Empty;

using TrajClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

using ros::Publisher;
using ros::Subscriber;
using ur10e_messages::State;


class RobotController
{
public:
    const double delta = 0.1;
    uint state = State::uninitialized;
    Publisher state_pub;
    State state_msg;

    EventsManager events;
    RobotModel robot_model;
    int robot_mode = -100;

    // ROS messaging
    TrajClient* traj_client;
    TrajGoal traj_goal;

    TrajState::ConstPtr joints_msg;
    Joy::ConstPtr joy_msg;

    Vec6 cart_vel_goal;
    int  gripper_command;

    Vec6 cur_joint_pos; // from state callback
    Vec3 cur_cart_pos_goal; // cartesian
    Mat3 cur_cart_rot_goal; // cartesian

    //--- construction and destruction
    RobotController();
    ~RobotController();

    //--- control functions
    void control(const ros::TimerEvent&);
    void init_control();
    void homing_control();
    void velocity_control();
    void position_control();

    //--- utility functions
    bool validate_and_send(Vec6& joints);
    bool validate_and_send(Vec6& joints, double dt);

    //--- message callbacks
    void state_callback(const TrajState::ConstPtr& pos);
    void joy_callback(const Joy::ConstPtr& joy);
    void robot_mode_callback(const RobotMode::ConstPtr& msg);

    //--- service callbacks
    bool init(Empty::Request&, Empty::Response&);
    bool stop_trajectory(Empty::Request&, Empty::Response&);
    bool start_velocity(Empty::Request&, Empty::Response&);
    bool start_home(Empty::Request&, Empty::Response&);
    bool start_position(Empty::Request&, Empty::Response&);
};

void print(const char* prefix, const Vec3& v);
void print(const char* prefix, const Vec6& v);
void print(const char* prefix, const Vec12& v);
void print(const char* prefix, const Mat3& m);
void print(const char* prefix, const Command& cmd);

Vec6 to_deg(const Vec6& q);

