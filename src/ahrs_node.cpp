/*
* @file      ahrs_node.cpp
* @date      11/3/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     The ROS node that utilizes the RazorAHRS object and publishes IMU data
*/

// Included Libraries
#include <string>

// 3rd Party Libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// User Libraries
#include "RazorAHRS.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"

int main(int argc, char **argv)
{
    // Create the AHRS instance
    RazorAHRS imu;
    imu.init();

    ros::init(argc, argv, "ahrs_node");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<picopter::Imu_msg>("imu_data", 1);

    ros::Rate loop_rate(100);

    // This is a hack to compute rates until I re-write the AHRS code to send rates over i2c
    imu.process();
    float pitch_last = imu.pitch;
    float roll_last = imu.roll;
    float yaw_last = imu.yaw;
    float pitch_rate_last = 0.0;
    float roll_rate_last = 0.0;
    float yaw_rate_last = 0.0;
    bool check = false;

    while (ros::ok)
    {
        picopter::Imu_msg msg;
        imu.process();
        msg.pitch = imu.pitch;
        msg.roll = imu.roll;
        msg.yaw = imu.yaw;
        msg.pitch_rate = (imu.pitch - pitch_last) * 100.0;
        msg.roll_rate = (imu.roll - roll_last) * 100.0;
        msg.yaw_rate = (yaw_last - imu.yaw) * 100.0;
        msg.pitch_rate_rate = (msg.pitch_rate - pitch_rate_last) * 100.0;
        msg.roll_rate_rate = (msg.roll_rate - roll_rate_last) * 100.0;
        msg.yaw_rate_rate = (msg.yaw_rate - yaw_rate_last) * 100.0;
        pitch_last = imu.pitch;
        roll_last = imu.roll;
        yaw_last = imu.yaw;
        pitch_rate_last = msg.pitch_rate;
        roll_rate_last = msg.roll_rate;
        yaw_rate_last = msg.yaw_rate;

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}