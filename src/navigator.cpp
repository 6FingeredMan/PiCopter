/*
* @file      Navigator.cpp
* @date      11/12/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Navigator Class
*/

// Included Libraries
#include <iostream>
#include <fstream>
#include <map>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "navigator.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Gps_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Altitude_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

Navigator::Navigator()
:
    RUN_OK(false),
    course_target(0.0),
    elevation_target(0.0),
    speed_target(0.0),
    altitude(0.0),
    min_altitude(0.0),
    latitude(0.0),
    longitude(0.0),
    northings(0.0),
    eastings(0.0),
    idle_status(true),
    objective_number(1),
    last_objective(1),
    curObjHeader("Objective "),
    curObj("Idle"),
    reader("/root/ros_catkin_ws/src/picopter/config/mission.ini")
{
    missionCheck();
    startNavigator();
}

void Navigator::missionCheck(void)
{
    // Check if the mission file exists
    if (reader.ParseError() != 0)
    {
        RUN_OK = false; // prevent the quad from running
        createMissionTemplate(); // create a basic mission file
    }

    // TO DO - verify all objectives are SAT
}

void Navigator::startNavigator(void)
{
    // First, fire up the subscribers with their callbacks
    gps_sub = n.subscribe<picopter::Gps_msg>("gps_data", 1, &Navigator::setLatLon, this);
    altitude_sub = n.subscribe<picopter::Altitude_msg>("altitude_data", 1, &Navigator::setAltitude, this);

    // Runs the main process
    nav_pub = n.advertise<picopter::Navigator_msg>("nav_data", 1);
    ros::Rate loop_rate(50);
    while (ros::ok)
    {
        process();
        nav_msg.target_course = course_target;
        nav_msg.target_elevation = elevation_target;
        nav_msg.target_speed = speed_target;
        nav_msg.idle = idle_status;
        nav_msg.current_objective = curObj;
        nav_msg.objectives_remaining = last_objective - objective_number;
        nav_pub.publish(nav_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Navigator::setLatLon(const picopter::Gps_msg::ConstPtr& msg)
{
    latitude = msg->lat;
    longitude = msg->lon;
    northings = msg->northings;
    eastings = msg->eastings;
}

void Navigator::setAltitude(const picopter::Altitude_msg::ConstPtr& msg)
{
    altitude = msg->altitude;
}

void Navigator::setTargetStates(void)
{
    // TO DO
}

void Navigator::process(void)
{
    // Check to see if the quad is at the end of the mission
    // and force an idle state if it is
    if(objective_number > last_objective)
    {
        idle();
        return;
    }
    
    // Parse through the mission file and call the 
    // appropriate objective function
    curObjHeader = "Objective " + std::to_string(objective_number);
    curObj = reader.Get(curObjHeader, "type", "UNKNOWN");

    // This super hacky - TO DO - use a map and stop
    // reading from the mission.ini every damn loop... it's slow.
    if(curObj == "Idle")
    {
        idle();
        return;
    }
    if(curObj == "Hover")
    {
        hover();
        return;
    }
    if(curObj == "Hover Rotate")
    {
        hoverRotate();
        return;
    }
    if(curObj == "Hold Station")
    {
        holdStation();
        return;
    }
    if(curObj == "Hold Station Rotate")
    {
        holdStationRotate();
        return;
    }
    if(curObj == "Navigate")
    {
        navigate();
        return;
    }
    if(curObj == "Land")
    {
        land();
        return;
    }
}

void Navigator::createMissionTemplate(void)
{
    std::ofstream tempFile("/root/ros_catkin_ws/src/picopter/config/mission.ini");
	if (tempFile.is_open())
	{
        tempFile << "[Objective 1]\n";
        tempFile << "Type = Idle\n";
        tempFile << "Duration (minutes) = 0.5\n";
        tempFile << "\n";

        tempFile << "[Objective 2]\n";
        tempFile << "Type = Hover\n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 3]\n";
        tempFile << "Type = Hover Rotate\n";
        tempFile << "Rate (deg/s) = 10\n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 4]\n";
        tempFile << "Type = Hold Station\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 5]\n";
        tempFile << "Type = Hold Station Rotate\n";
        tempFile << "Rate (deg/s) = 10\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Duration (minutes) = 1.0\n";
        tempFile << "\n";

        tempFile << "[Objective 6]\n";
        tempFile << "Type = Navigate\n";
        tempFile << "Latitude = \n";
        tempFile << "Longitude = \n";
        tempFile << "Transit Elevation (meters) = 10\n";
        tempFile << "Destination Elevation (meters) = 10\n";
        tempFile << "Minimum Altitude (meters) = 1.0\n";
        tempFile << "Orientation = Any\n";
        tempFile << "Speed (m/s) = 1\n";
        tempFile << "\n";

        tempFile << "[Objective 7]\n";
        tempFile << "Type = Land\n";
        tempFile << "\n";

        tempFile << "[Objective 8]\n";
        tempFile << "Type = Idle\n";
        tempFile << "Duration (minutes) = 0.5\n";
        tempFile << "\n";

        tempFile.close();
    }
}

void Navigator::idle(void)
{
   idle_status = true;
}

void Navigator::hover(void)
{
    // TO DO
}

void Navigator::hoverRotate(void)
{
    // TO DO
}

void Navigator::holdStation(void)
{
    // TO DO
}

void Navigator::holdStationRotate(void)
{
    // TO DO
}

void Navigator::navigate(void)
{
    // TO DO
}

void Navigator::land(void)
{
    // TO DO
}


