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
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "autopilot.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

// Macros
#define PI 3.14159265

Autopilot::Autopilot()
:
    _controllerPrefix(),
    _controllerConfigType(),
    _controllers(),
    M1_cmd(0.0),
    M2_cmd(0.0),
    M3_cmd(0.0),
    M4_cmd(0.0),
    elevation_target(0.0),
    pitch_target(0.0),
    roll_target(0.0),
    yaw_target(0.0),
    pitch_val(0.0),
    roll_val(0.0),
    pitch_cmd(0.0),
    roll_cmd(0.0),
    yaw_cmd(0.0),
    z_cmd(0.0),
    idle_status(true)
{
    _controllerPrefix[altitude] = "Altitude Control";
    _controllerPrefix[speed] = "Speed Control";
    _controllerPrefix[pitch] = "Pitch Control";
    _controllerPrefix[roll] = "Roll Control";
    _controllerPrefix[yaw] = "Yaw Control";
    _controllerPrefix[pitch_rate] = "Pitch Rate Control";
    _controllerPrefix[roll_rate] = "Roll Rate Control";
    _controllerPrefix[yaw_rate] = "Yaw Rate Control";

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
    ahrs_sub = n.subscribe<picopter::Imu_msg>("imu_data", 1, &Autopilot::setAttitudeStates, this);

    // Updates the idle status and zeroizes motor commands and integrators if true
    nav_sub = n.subscribe<picopter::Navigator_msg>("nav_data", 1, &Autopilot::setNavStates, this);

    // Runs the main process
    motor_pub = n.advertise<picopter::Motors_msg>("motor_cmds", 1);
    ros::Rate loop_rate(50);
    while (ros::ok)
    {
        // If the idle status is false, 
        // process the control loops and then 
        // publish the controller motor commands
        if(idle_status == false)
        {
            processLoops();
            motor_msg.M1 = M1_cmd;
            motor_msg.M2 = M2_cmd;
            motor_msg.M3 = M3_cmd;
            motor_msg.M4 = M4_cmd;
        }
        // If the idle status is true, zeroize the motor commands 
        // and prevent integrator windup by NOT processing the control loops.
        // NOT processing the control loops stil preserves the GOOD integrator.
        else 
        {
            motor_msg.M1 = 0.0;
            motor_msg.M2 = 0.0;
            motor_msg.M3 = 0.0;
            motor_msg.M4 = 0.0;
        }
        motor_pub.publish(motor_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void Autopilot::setAttitudeStates(const picopter::Imu_msg::ConstPtr& msg)
{
    _controllers[pitch]->setStates(msg->pitch, msg->pitch_rate);
    _controllers[roll]->setStates(msg->roll, msg->roll_rate);
    _controllers[yaw]->setStates(msg->yaw, msg->yaw_rate);
    pitch_val = msg->pitch;
    roll_val = msg->roll;
}

void Autopilot::setNavStates(const picopter::Navigator_msg::ConstPtr& msg)
{
    elevation_target = msg->target_elevation;
    yaw_target = msg->target_course;
    idle_status = msg->idle;
}

void Autopilot::setElevationState(void)
{
    _controllers[altitude]->setStates(0.0, 0.0);
}

void Autopilot::setTargets(void)
{
    // TEMPORARY
    _controllers[altitude]->setTarget(0.0);
    _controllers[pitch]->setTarget(0.0);
    _controllers[roll]->setTarget(0.0);
    _controllers[yaw]->setTarget(0.0);
}

void Autopilot::processLoops(void)
{
    //std::cout<< "Looping!" <<std::endl;

    // Process the positional controllers first
    _controllers[altitude]->process();
    _controllers[speed]->process();

    // Pass the output of the north and east controllers as input to the attitude controllers
    // TO DO

    // Process the control loops
    _controllers[pitch]->process();
    _controllers[roll]->process();
    _controllers[yaw]->process();

    pitch_cmd = _controllers[pitch]->returnCmd();
    roll_cmd = _controllers[roll]->returnCmd();
    yaw_cmd = _controllers[yaw]->returnCmd();
    z_cmd = _controllers[altitude]->returnCmd();

    mix();
}

void Autopilot::mix(void)
{
    // Mixes the output commands from the attitude controllers
    // and updates the motor_cmd message before publishing to the topic.

    // Step 1 - compensate for pitch and roll to correct the altitude command...
    // in theory this lets the altitude controller integrator compensate for battery drain
    z_cmd = z_cmd / ( cos(abs(pitch_val) * PI / 180.0) * cos(abs(roll_val) * PI / 180.0) );

    if(z_cmd > 100.0)
    {
        z_cmd = 100.0;
    }
    if(z_cmd < 0.0)
    {
        z_cmd = 0.0;
    }

    M1_cmd = z_cmd;
    M2_cmd = z_cmd;
    M3_cmd = z_cmd;
    M4_cmd = z_cmd;

    // Step 2 - add the pitch command
    M1_cmd += pitch_cmd;
    M2_cmd -= pitch_cmd;
    M3_cmd -= pitch_cmd;
    M4_cmd += pitch_cmd;

    // Step 3 -- add the roll command
    M1_cmd += roll_cmd;
    M2_cmd += roll_cmd;
    M3_cmd -= roll_cmd;
    M4_cmd -= roll_cmd;

    // Step 4 -- add the yaw command
    M1_cmd -= yaw_cmd;          // CW Prop, (-) yaw torque
    M2_cmd += yaw_cmd;          // CCW Prop, (+) yaw torque
    M3_cmd -= yaw_cmd;          // CW Prop, (-) yaw torque
    M4_cmd += yaw_cmd;          // CCW Prop, (+) yaw torque
}