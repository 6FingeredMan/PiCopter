/*
* @file      ahrs_sim.h
* @date      12/5/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the AHRS simulator class
*/
#ifndef __AHRS_SIM_H__
#define __AHRS_SIM_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include <ros/ros.h>

// User Libraries
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Sim_msg.h"

// Macros

class AHRS_SIM
{
public:

	// Constructor
	AHRS_SIM();

    // Destructor
    //~Autopilot();

	// Load Components
	void loadConfig(void);

	// Start the ROS processing loops
	void startAhrs(void);

	// Sets current attitude states for the controllers
	void setStates(const picopter::Sim_msg::ConstPtr& msg);

	// Public variable declarations
	picopter::Imu_msg ahrs_msg;
    float pitch;
    float roll;
    float yaw;
    float pitch_noise;
    float roll_noise;
    float yaw_noise;
    float pitch_rate_noise;
    float roll_rate_noise;
    float yaw_rate_noise;
    float sim_rate;
	float R2D;

private:

	// ROS hooks
	ros::NodeHandle n;
	ros::Publisher ahrs_pub;
	ros::Subscriber sim_sub;
	
};

#endif // __AHRS_SIM_H__