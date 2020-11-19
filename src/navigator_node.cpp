/*
* @file      navigator_node.cpp
* @date      11/16/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that utilizes the Navigator object.
*            Subcribes to the following topics:
*            - GPS
*            - Elevation
*            - Altitude
*            - Simulator
*            - 
*            Publishes to the following topics:
*            - TargetStates
*            
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "ros/ros.h"

// User Libraries
#include "navigator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigator_node");

    // Create the AHRS instance
    Navigator captain;

    ros::spin();

    return 0;
}