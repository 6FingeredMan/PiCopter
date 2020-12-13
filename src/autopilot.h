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
#include "/root/ros_catkin_ws/devel/include/picopter/Autopilot_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Elevation_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

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
	void setAttitudeStates(const picopter::Imu_msg::ConstPtr& msg);

	// Sets the navigation states for the controllers
	void setNavStates(const picopter::Navigator_msg::ConstPtr& msg);

	// Converts a target course & speed to pitch & roll targets
	void convertNavToAttitude(void);

	// Sets elevation state for the controller
	void setElevationState(const picopter::Elevation_msg::ConstPtr& msg);

	// Sets target states for the controllers
	void setTargets(void);

	// Processes all of the control loops
	void processLoops(void);

	// Handles controller mixing and priority of controls
	// and computes motor commands
	void mix(void);

	// Checks to see if any controllers are saturating the throttle
	// command below zero or above 100
	void saturate(void);

	// Checks for NaN output and sets motors to zero if true
	void checkNaN(void);

	// Collects Autopilot data to publish
	void collectAutopilotData(void);

	float limitZero(float input);

	float limit(float input, float limit);

	// Public variable declarations
	float M1_cmd;
	float M2_cmd;
	float M3_cmd;
	float M4_cmd;
	float elevation_target;
	float pitch_target;
	float roll_target;
	float yaw_target;
	float pitch_val;
	float roll_val;
	float yaw_val;
	float pitch_cmd;
	float roll_cmd;
	float yaw_cmd;
	float z_cmd;
	bool idle_status;
	float elevation;
	float elevation_last;
	float elevation_rate;
	float elevation_rate_last;
	float elevation_accel;
	float R2D;
	float D2R;
	float controller_frequency;
	bool navigator_override;
	picopter::Motors_msg motor_msg;
	picopter::Autopilot_msg pilot_msg;



private:

	// ROS hooks
	ros::NodeHandle n;
	ros::Publisher motor_pub;
	ros::Publisher pilot_pub;
	ros::Subscriber ahrs_sub;
	ros::Subscriber nav_sub;
	ros::Subscriber elevation_sub;

	// Controller Factory Hooks
	enum {altitude, speed, pitch, roll, yaw};

	std::map< uint32_t, std::string > _controllerPrefix;
    std::map< uint32_t, std::string > _controllerConfigType;
    std::map< uint32_t, ControllerInterface *> _controllers;

	
};

#endif // __AUTOPILOT_H__