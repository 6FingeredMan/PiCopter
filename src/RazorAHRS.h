/*
* @file      RazorAHRS.h
* @date      10/31/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the RazorAHRS Interface Class
*/
#ifndef __RAZOR_AHRS_H__
#define __RAZOR_AHRS_H__

// Included Libraries
#include <cinttypes>

// 3rd Party Libraries

// User Libraries

class RazorAHRS
{
    public:
        bool init(void);
        void process(void);
        void debug(int delay_time);
        float pitch;
        float roll;
        float yaw;
        float pitch_rate;
        float roll_rate;
        float yaw_rate;

    private:
        int16_t int_data[3];
        uint8_t  device;
        const char ADDRESS = 0x09;
};


#endif // __RAZOR_AHRS_H__