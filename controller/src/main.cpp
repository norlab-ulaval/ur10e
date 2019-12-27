#include <ros/ros.h>


#include "linear.h"
#include "robot_controller.h"
#include "robot_model.h"



using RC = RobotController;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR10e_controller");
    ros::NodeHandle node;
    RC rc;

    //--- advertise services
    auto ss2 = node.advertiseService("ur10e/stop_traj", &RC::stop_trajectory,   &rc);
    auto ss3 = node.advertiseService("ur10e/start_vel", &RC::start_velocity,    &rc);
    auto ss4 = node.advertiseService("ur10e/start_pos", &RC::start_position,    &rc);
    auto ss5 = node.advertiseService("ur10e/init",      &RC::init,              &rc);
    auto ss6 = node.advertiseService("ur10e/home",      &RC::start_home,        &rc);
    auto ss7 = node.advertiseService("ur10e/reset_errors", &RC::reset_errors,   &rc);

    //--- advertise topics
    rc.state_pub = node.advertise<State>("ur10e/state", 0);

    //--- subscribe to topics
    auto sub1 = node.subscribe<TrajState>("/scaled_pos_traj_controller/state", 0, &RC::state_callback,
            &rc);
    auto sub2 = node.subscribe<Joy>("joy", 0, &RC::joy_callback, &rc);
    auto sub3 = node.subscribe<RobotMode>("/ur_hardware_interface/robot_mode", 0,
            &RC::robot_mode_callback, &rc);

    //--- create timer for main control loop
    ros::Timer timer = node.createTimer(ros::Duration(0.1), &RC::control, &rc);

    // todo: should this be multi-threaded?
    ros::spin();
}
