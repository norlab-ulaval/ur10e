#pragma once

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/Joy.h>
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
    standstill,
    cart_velocity,
    stopping,
    error,
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

    Publisher   command_pub;
    MsgTrajGoal command_msg;

    Publisher     cancel_pub;
    MsgTrajCancel cancel_msg;

    Vec6 cur_joint_pos; // from state callback
    Vec3 cur_cart_pos_goal; // cartesian
    Mat3 cur_cart_rot_goal; // cartesian

    //---- construction and destruction ----
    RobotController();

    //---- control functions ----
    void control();
    void velocity_control();
    bool stopping_control();

    //---- callbacks ----
    void state_callback(const MsgTrajState::ConstPtr& pos);
    void joy_callback(const MsgJoy::ConstPtr& joy);
};



void print(const char* prefix, const Vec3& v);
void print(const char* prefix, const Vec6& v);
void print(const char* prefix, const Vec12& v);
void print(const char* prefix, const Mat3& m);
void print(const char* prefix, const State& state);
void print(const char* prefix, const Command& cmd);

Vec6 to_deg(const Vec6& q);

