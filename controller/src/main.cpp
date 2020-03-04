#include <ros/ros.h>


#include "ur10e_linear.h"
#include "ur10e_controller.h"
#include "ur10e_model.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR10e_controller");
    ros::NodeHandle node;
    using Controller = Ur10eController;
    Controller controller;

    //--- advertise services
    auto ss1 = node.advertiseService("ur10e/stop_traj",     &Controller::stop_trajectory, &controller);
    auto ss2 = node.advertiseService("ur10e/start_vel",     &Controller::start_velocity,  &controller);
    auto ss3 = node.advertiseService("ur10e/start_pos",     &Controller::start_position,  &controller);
    auto ss4 = node.advertiseService("ur10e/init",          &Controller::init,            &controller);
    auto ss5 = node.advertiseService("ur10e/home",          &Controller::start_home,      &controller);
    auto ss6 = node.advertiseService("ur10e/reset_errors",  &Controller::reset_errors,    &controller);
    // other services
    // - add position to queue
    // - replace position by new goal
    // - query current goal(s)
    // - cartesian motions

    //--- advertise topics
    controller.state_pub = node.advertise<State>("ur10e/state", 0);

    //--- subscribe to topics
    auto sub1 = node.subscribe<TrajState>("/scaled_pos_traj_controller/state", 0, &Controller::state_callback, &controller);
    auto sub2 = node.subscribe<Joy>("joy", 0, &Controller::joy_callback, &controller);
    auto sub3 = node.subscribe<RobotMode>("/ur_hardware_interface/robot_mode", 0, &Controller::robot_mode_callback, &controller);
    auto sub4 = node.subscribe<std_msgs::Float64>("/speed_scaling_factor", 0, &Controller::speed_scale_callback, &controller);

    //--- create timer for main control loop
    ros::Timer timer = node.createTimer(ros::Duration(0.1), &Controller::control, &controller);


    // todo: should this be multi-threaded?
    ros::spin();
}

