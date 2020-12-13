/*
* @file      ahrs_sim.cpp
* @date      11/4/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Autopilot Class
*/

// Included Libraries
#include <iostream>
#include <map>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "ahrs_sim.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

// Macros

AHRS_SIM::AHRS_SIM()
:
pitch(0.0),
roll(0.0),
yaw(0.0),
pitch_noise(0.0),
roll_noise(0.0),
yaw_noise(0.0),
pitch_rate_noise(0.0),
roll_rate_noise(0.0),
yaw_rate_noise(0.0),
sim_rate(100.0),
R2D(57.2958)
{
    loadConfig();
    startAhrs();
}

void AHRS_SIM::loadConfig(void)
{
    // Declare an ini reader instance and pass it the location of the ini file
    INIReader reader("/root/ros_catkin_ws/src/picopter/config/simulation.ini");
    pitch_noise = reader.GetReal("AHRS", "pitch noise amplitude (deg)", 0.0);
    pitch_noise = pitch_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    roll_noise = reader.GetReal("AHRS", "roll noise amplitude (deg)", 0.0);
    roll_noise = roll_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    yaw_noise = reader.GetReal("AHRS", "yaw noise amplitude (deg)", 0.0);
    yaw_noise = yaw_noise * 2.0; // To get proper amplitude value since noise is generated with rand & +/- 0.5
    sim_rate = reader.GetReal("AHRS", "update rate (Hz)", 100.0);
}

void AHRS_SIM::startAhrs(void)
{
    // First, fire up the subscriber to the sim data with a callback to update the attitude values
    sim_sub = n.subscribe<picopter::Sim_msg>("sim_data", 1, &AHRS_SIM::setStates, this);

    // Run the main loop that publishes simulated AHRS data to the "imu_data" topic
    ahrs_pub = n.advertise<picopter::Imu_msg>("imu_data", 1);
    ros::Rate loop_rate(sim_rate);

    float pitch_last = pitch;
    float roll_last = roll;
    float yaw_last = yaw;
    float pitch_rate_last = 0.0;
    float roll_rate_last = 0.0;
    float yaw_rate_last = 0.0;

    while (ros::ok)
    {
        // Since the sim data values are perfect without noise, use rand 
        // to generate a normally distributed random number between 0 and 1,
        // then shift it to +/- 0.5, and multiply by the 2x the noise amplitude
        ahrs_msg.pitch = pitch + ((((float) rand() / RAND_MAX)) - 0.5)*pitch_noise;
        ahrs_msg.roll = roll + ((((float) rand() / RAND_MAX)) - 0.5)*roll_noise;
        ahrs_msg.yaw = yaw + ((((float) rand() / RAND_MAX)) - 0.5)*yaw_noise;

        // Calculate the rates based on state measurements and update rate
        ahrs_msg.pitch_rate = (pitch - pitch_last) * sim_rate;
        ahrs_msg.roll_rate = (roll - roll_last) * sim_rate;
        ahrs_msg.yaw_rate = (yaw_last - yaw) * sim_rate;
        ahrs_msg.pitch_rate_rate = (ahrs_msg.pitch_rate - pitch_rate_last) * sim_rate;
        ahrs_msg.roll_rate_rate = (ahrs_msg.roll_rate - roll_rate_last) * sim_rate;
        ahrs_msg.yaw_rate_rate = (ahrs_msg.yaw_rate - yaw_rate_last) * sim_rate;
        pitch_last = pitch;
        roll_last = roll;
        yaw_last = yaw;
        pitch_rate_last = ahrs_msg.pitch_rate;
        roll_rate_last = ahrs_msg.roll_rate;
        yaw_rate_last = ahrs_msg.yaw_rate;

        ahrs_pub.publish(ahrs_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void AHRS_SIM::setStates(const picopter::Sim_msg::ConstPtr& msg)
{
    pitch = msg->pitch * R2D;
    roll = msg->roll * R2D;
    yaw = msg->yaw * R2D;
}
