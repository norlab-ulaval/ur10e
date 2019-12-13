#include <ros/ros.h>


#include "linear.h"
#include "robot_controller.h"
#include "robot_model.h"





int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR10e_controller");
    ros::NodeHandle node;
    RobotController robot_controller;

    //--- advertise to topics
    robot_controller.command_pub = node.advertise<MsgTrajGoal>(
            "scaled_pos_traj_controller/follow_joint_trajectory/goal", 1);
    robot_controller.cancel_pub = node.advertise<MsgTrajCancel>(
            "scaled_pos_traj_controller/follow_joint_trajectory/cancel", 1);
    robot_controller.state_pub = node.advertise<std_msgs::UInt16>(
            "ur10e/state", 1);

    //--- subscribe to topics
    //     node.subscribe<MsgTrajState>(
    robot_controller.joints_sub = node.subscribe<MsgTrajState>(
            "/scaled_pos_traj_controller/state", 10, &RobotController::state_callback,
            &robot_controller);

    robot_controller.joy_sub = node.subscribe<MsgJoy>(
            "joy", 10, &RobotController::joy_callback, &robot_controller);

    robot_controller.start_vel_sub = node.subscribe<std_msgs::Empty>(
            "ur10e/start_vel", 1, &RobotController::start_vel_callback, &robot_controller);

    robot_controller.stop_sub = node.subscribe<std_msgs::Empty>(
            "ur10e/stop", 1, &RobotController::stop_callback, &robot_controller);


    //     Subscriber reset_sub;
    //     Subscriber stop_sub;
    //     Subscriber start_vel_sub;
    //     Subscriber start_pos_sub;

    // todo: should this be multi-threaded?
    ros::spin();
}
