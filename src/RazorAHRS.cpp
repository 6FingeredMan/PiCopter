/*
* @file      RazorAHRS.cpp
* @date      10/31/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the RazorAHRS Interface Class
*/

// Included Libraries
#include <cinttypes>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>   
#include <unistd.h>
#include <errno.h>
#include "string.h"
#include <math.h>
#include <iomanip>

// 3rd Party Libraries

// User Libraries
#include "RazorAHRS.h"

bool RazorAHRS::init(void)
{
    // Open the i2c bus and verify the device exists and can be slaved
    device = open("/dev/i2c-1", O_RDWR);
    if (ioctl(device, I2C_SLAVE, ADDRESS) == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
    //close(device);
    
}

void RazorAHRS::process(void)
{
    //device = open("/dev/i2c-1", O_RDWR);
    read(device, int_data, sizeof(int_data));
    yaw = int_data[0] / 90.0;
    pitch = int_data[1] / 100.0;
    roll = int_data[2] / 100.0;
    //close(device);

}

void RazorAHRS::debug(int delay_time)
{
    process();
    std::cout << std::fixed << std::setprecision(3); 
    std::cout << "Pitch: " << pitch << " Roll: " << roll << " Yaw: " << yaw << "\r" << std::flush;
    usleep(delay_time);
}