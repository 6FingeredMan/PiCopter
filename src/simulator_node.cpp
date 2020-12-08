/*
* @file      simulator_node.cpp
* @date      12/5/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that utilizes the Simulator object and subscribes to motor commands and publishes simulated state data.
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "ros/ros.h"

// User Libraries
#include "simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulator_node");

    // Create the AHRS instance
    Simulator sim;

    ros::spin();

    return 0;
}