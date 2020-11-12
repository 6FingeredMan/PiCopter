/*
* @file      autopilot_node.cpp
* @date      11/10/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that utilizes the Autopilot object and subscribes to imu and altitude data and publishes to motor commands.
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// User Libraries
#include "autopilot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autopilot_node");

    // Create the AHRS instance
    Autopilot pilot;

    ros::spin();

    return 0;
}