/*
* @file      ahrs_sim_node.cpp
* @date      11/3/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that simulates the RazorAHRS object and publishes IMU data based on simulation data
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// User Libraries
#include "ahrs_sim.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Sim_msg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ahrs_sim_node");

    // Create the AHRS simulation instance
    AHRS_SIM ahrs;

    ros::spin();

    return 0;
}