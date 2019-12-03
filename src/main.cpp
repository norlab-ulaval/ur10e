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
    robot_controller.command_pub = node.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
            "scaled_pos_traj_controller/follow_joint_trajectory/goal", 1);
    robot_controller.cancel_pub = node.advertise<actionlib_msgs::GoalID>(
            "scaled_pos_traj_controller/follow_joint_trajectory/cancel", 1);

    //--- subscribe to topics
    robot_controller.joints_sub = node.subscribe<control_msgs::JointTrajectoryControllerState>(
            "/scaled_pos_traj_controller/state", 10, &RobotController::state_callback,
            &robot_controller);
    robot_controller.joy_sub = node.subscribe<sensor_msgs::Joy>(
            "joy", 10, &RobotController::joy_callback, &robot_controller);

    // todo(andre): should this be multi-threaded?
    ros::spin();
}
