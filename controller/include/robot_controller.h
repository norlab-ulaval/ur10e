#pragma once

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "robot_model.h"
#include "robot_events.h"
#include "linear.h"

const double angular_speed = 1.0; // rad/s
const double linear_speed = 0.2;  // m/s

using MsgTrajGoal   = control_msgs::FollowJointTrajectoryActionGoal;
using MsgTrajCancel = actionlib_msgs::GoalID;
using MsgTrajState  = control_msgs::JointTrajectoryControllerState;
using MsgJoy        = sensor_msgs::Joy;

using ros::Publisher;
using ros::Subscriber;


enum class State
{
    standstill = 1,
    cart_velocity = 10,
    joint_position = 20,
    stopping = 100,
    error = 200,
};

struct RobotController
{
    State           state = State::standstill;
    EventsManager   events;
    RobotModel      robot_model;

    // ROS messaging
    Subscriber             joints_sub;
    MsgTrajState::ConstPtr joints_msg;

    Subscriber       joy_sub;
    MsgJoy::ConstPtr joy_msg;
    Vec6 cart_vel_goal;
    int  gripper_command;

    Publisher        state_pub;
    std_msgs::UInt16 state_msg;

    Publisher        command_pub;
    MsgTrajGoal      command_msg;

    Publisher     cancel_pub;
    MsgTrajCancel cancel_msg;

    // ros triggers from the GUI
    Subscriber reset_sub;
    Subscriber stop_sub;
    Subscriber start_vel_sub;
    Subscriber start_pos_sub;

    Vec6 cur_joint_pos; // from state callback
    Vec3 cur_cart_pos_goal; // cartesian
    Mat3 cur_cart_rot_goal; // cartesian

    //---- construction and destruction ----
    RobotController();

    //---- control functions ----
    void control();
    void velocity_control();
    void position_control();
    bool stopping_control();

    //---- callbacks ----
    void state_callback(const MsgTrajState::ConstPtr& pos);
    void joy_callback(const MsgJoy::ConstPtr& joy);
    void stop_callback(const std_msgs::Empty::ConstPtr& )
    {
        events.add(Command::stop);
    };
    void start_vel_callback(const std_msgs::Empty::ConstPtr& )
    {
        events.add(Command::start_cart_velocity);
    };
    void start_pos_callback(const std_msgs::Empty::ConstPtr& ) {};
};

void print(const char* prefix, const Vec3& v);
void print(const char* prefix, const Vec6& v);
void print(const char* prefix, const Vec12& v);
void print(const char* prefix, const Mat3& m);
void print(const char* prefix, const State& state);
void print(const char* prefix, const Command& cmd);

Vec6 to_deg(const Vec6& q);

