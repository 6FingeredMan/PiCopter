/*
* @file      Autopilot.h
* @date      11/4/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Autopilot Class
*/
#ifndef __AUTOPILOT_H__
#define __AUTOPILOT_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include <ros/ros.h>

// User Libraries
#include "Controllers.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors.h"

class Autopilot
{
public:

	// Constructor
	Autopilot();

    // Destructor
    //~Autopilot();

	// Load Components
	void loadConfig(void);

	// Start the ROS processing loops
	void startAutopilot(void);

	// Sets current attitude states for the controllers
	void setAttitudeStates(const picopter::Imu::ConstPtr& msg);

	// Sets elevation state for the controller

	// Sets target states for the controllers
	void setTargets(void);

	// Processes all of the control loops
	void processLoops(void);

	// Handles controller mixing and priority of controls
	// and computes motor commands
	void mix(void);
	float M1_cmd;
	float M2_cmd;
	float M3_cmd;
	float M4_cmd;
	picopter::Motors motor_msg;



private:

	// ROS hooks
	ros::NodeHandle n;
	ros::Publisher motor_pub;
	ros::Subscriber ahrs_sub;

	// Controller Factory Hooks
	enum {altitude, north, east, pitch, roll, yaw};

	std::map< uint32_t, std::string > _controllerPrefix;
    std::map< uint32_t, std::string > _controllerConfigType;
    std::map< uint32_t, ControllerInterface *> _controllers;

	
};

#endif // __AUTOPILOT_H__