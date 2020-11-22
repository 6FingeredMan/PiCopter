/*
* @file      Navigator.h
* @date      11/12/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Navigator Class
*/
#ifndef __NAVIGATOR_H__
#define __NAVIGATOR_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include <ros/ros.h>
#include "INIReader.h"

// User Libraries
#include "Controllers.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Gps_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Altitude_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

class Navigator
{
public:

	// Constructor
	Navigator();

    // Destructor
    //~Autopilot();

	// Checks the validity of the mission file
	void missionCheck(void);

	// Start the ROS processing loops
	void startNavigator(void);

    // Sets the current lat/lon and northings/eastings
    void setLatLon(const picopter::Gps_msg::ConstPtr& msg);

    // Sets the current altitude in meters
    void setAltitude(const picopter::Altitude_msg::ConstPtr& msg);

	// Processes the mission 
	void process(void);

    // Sets current target states for the controllers
	void setTargetStates(void);

    // Creates a basic mission file with all available behaviors
    void createMissionTemplate(void);

	// Public variable declarations
    bool RUN_OK = false;
    float course_target;
    float elevation_target;
    float speed_target;
    float altitude;
    float min_altitude;
    float heading_rate_target;
    float latitude;
    float longitude;
    float northings;
    float eastings;
    bool idle_status;
    int objective_number;
    int prev_objective_number;
    int end_objective_number;
    std::string curObjHeader;
    std::string curObj;
    picopter::Navigator_msg nav_msg;

    // Mission Reader
    INIReader reader;

    // Enum containing the list of available behaviors
    enum Objectives
    {
        IDLE,
        HOVER,
        HOVER_ROTATE,
        HOLD_STATION,
        HOLD_STATION_ROTATE,
        NAVIGATE,
        LAND
    };

private:

	// ROS hooks
	ros::NodeHandle n;
    ros::Publisher nav_pub;
    ros::Subscriber gps_sub;
    ros::Subscriber altitude_sub;

    // Behaviors / Objectives
    std::map< std::string, enum Objectives > ObjectivesToEnum;

    // Idle behavior - no outputs and waits for transition command
    void idle(void);

    // Hover behavior - holds steady elevation and zero attitude, does
    // not attempt to maintain a lat/lon
    void hover(void);

    // Hover rotate behavior - holds steady elevation and rotates about the yaw axis,
    // does not attempt to maintain a lat/lon
    void hoverRotate(void);

    // Hold station behavior - holds steady elevation and maintains lat/lon
    void holdStation(void);

    // Hold station rotate behavior - holds steady elevation and maintains lat/lon
    // while rotating about the yaw axis
    void holdStationRotate(void);

    // Navigate behavior - guides the quad to a waypoint
    void navigate(void);

    // Land behavior - lands the quad
    void land(void);
	
};

#endif // __NAVIGATOR_H__