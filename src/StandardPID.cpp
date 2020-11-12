/*
* @file      StandardPID.cpp
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines a standard PID controller algorithm
*/

// Included Libraries
#include <iostream>
#include <math.h>
#include <string>

// 3rd Party Libraries
#include "INIReader.h"

// User Libraries
#include "StandardPID.h"

StandardPID::StandardPID()
:
    Kp(0.0), Ki(0.0), Kd(0.0),
    I(0.0),
    maxI(0.0), max(0.0),
    target(0.0),
    error(0.0),
    state(0.0),
    state_rate(0.0),
    cmd(0.0),
    dt(0.0)
{
    reset();
}

void StandardPID::reset(void)
{
    Kp = Ki = Kd = 0.0;
    I = 0.0;
    maxI = max = 0.0;
    target = state = state_rate = cmd = 0.0;
    dt = 0.02;
}

void StandardPID::loadConfig(std::string & DOF)
{
    INIReader reader("/root/ros_catkin_ws/src/picopter/config/ini_test.ini");

    Kp = reader.GetReal(DOF, "Kp", -1);
    Ki = reader.GetReal(DOF, "Ki", -1);
    Kd = reader.GetReal(DOF, "Kd", -1);
    maxI = reader.GetReal(DOF, "maxI", -1);
    max = reader.GetReal(DOF, "max", -1);

    // DEBUG ONLY
    std::cout << "Created a " << DOF << " PID Controller." << std::endl;
    std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << std::endl;

}

void StandardPID::setTarget(float val)
{
    target = val;
}

void StandardPID::setStates(float val1, float val2)
{
    state = val1;
    state_rate = val2;
}


void StandardPID::process(void)
{
    error = target - state;

    I += Ki*error*dt;

    // Clamp the Integrator
    if(I > maxI)
    {
        I = maxI;
    }
    if(I < -maxI)
    {
        I = -maxI;
    }

    // Compute the command and clamp it
    cmd = Kp*error + Kd*state_rate + I;

    if(cmd > 100.0)
    {
        cmd = 100.0;
    }

}



