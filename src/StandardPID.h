/*
* @file      StandardPID.h
* @date      11/6/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines a standard linear PID controller algorithm
*/
#ifndef __STANDARD_PID_H__
#define __STANDARD_PID_H__

// Included Libraries
#include <string>

// 3rd Party Libraries

// User Libraries
#include "Controllers.h"

class StandardPID : public ControllerInterface
{
    public:
        StandardPID();
        void reset(void);
        void loadConfig(std::string & DOF);
        void setTarget(float val);
        void setStates(float val1, float val2);
        void process(void);
        float returnCmd(void);

    protected:
        float Kp, Ki, Kd;   // PID Gains
        float I;            // Integrator Sum
        float maxI, max;    // Integrator limit and maximum command
        float target;       // Target state value
        float error;        // State error value
        float state;        // Current state value
        float state_rate;   // Current state derivative (rate)
        float cmd;          // Output command (0-100%)
        float dt;           // 1/Hz (controller frequency in seconds)

};


#endif // __SUPER_TWISTING_SMC_H__