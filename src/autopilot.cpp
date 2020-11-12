/*
* @file      Autopilot.h
* @date      11/4/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Autopilot Class
*/

// Included Libraries
#include <iostream>
#include <map>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "autopilot.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors.h"

Autopilot::Autopilot()
:
    _controllerPrefix(),
    _controllerConfigType(),
    _controllers(),
    M1_cmd(0.0),
    M2_cmd(0.0),
    M3_cmd(0.0),
    M4_cmd(0.0)
{
    _controllerPrefix[altitude] = "Altitude Control";
    _controllerPrefix[north] = "Translation Control North";
    _controllerPrefix[east] = "Translation Control East";
    _controllerPrefix[pitch] = "Pitch Control";
    _controllerPrefix[roll] = "Roll Control";
    _controllerPrefix[yaw] = "Yaw Control";

    loadConfig();
    startAutopilot();
}

void Autopilot::loadConfig(void)
{
    // Declare an ini reader instance and pass it the location of the ini file
    INIReader reader("/root/ros_catkin_ws/src/picopter/config/ini_test.ini");
    std::string temp; // Used as a temporary string to store an ini string

    // Loop through all of the controllers, read their congfig type, and return 
    // a new instance of that controller type. Then load the config of the controller.
    for (auto iter = _controllerPrefix.begin(); iter != _controllerPrefix.end(); ++iter)
    {
        temp = reader.Get(iter->second, "type", "UNKOWN");
        _controllerConfigType[iter->first] = temp;
    }

    for(auto iter = _controllerConfigType.begin(); iter != _controllerConfigType.end(); ++iter)
    {
        _controllers[iter->first] = ControllerFactory::instance()->createController(iter->second);
        _controllers[iter->first]->loadConfig(_controllerPrefix[iter->first]);
    }
    
}

void Autopilot::startAutopilot(void)
{

    // First, fire up the subscribers with their callbacks

    // Updates the attitude states - pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate
    ahrs_sub = n.subscribe<picopter::Imu>("imu_data", 1, &Autopilot::setAttitudeStates, this);

    // Runs the main process
    motor_pub = n.advertise<picopter::Motors>("motor_cmds", 1);
    ros::Rate loop_rate(50);
    while (ros::ok)
    {
        processLoops();
        motor_msg.M1 = M1_cmd;
        motor_msg.M2 = M2_cmd;
        motor_msg.M3 = M3_cmd;
        motor_msg.M4 = M4_cmd;
        motor_pub.publish(motor_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void Autopilot::setAttitudeStates(const picopter::Imu::ConstPtr& msg)
{
    std::cout<< "Setting Attitude States!" <<std::endl;
    _controllers[pitch]->setStates(msg->pitch, msg->pitch_rate);
    _controllers[roll]->setStates(msg->roll, msg->roll_rate);
    _controllers[yaw]->setStates(msg->yaw, msg->yaw_rate);
}

void Autopilot::setTargets(void)
{
    // TO DO
}

void Autopilot::processLoops(void)
{
    //std::cout<< "Looping!" <<std::endl;
    _controllers[pitch]->process();
    _controllers[roll]->process();
    _controllers[yaw]->process();
}

void mix(void)
{
    // TO DO
}