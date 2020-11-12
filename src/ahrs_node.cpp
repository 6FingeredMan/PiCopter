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
#include "/root/ros_catkin_ws/devel/include/picopter/Imu.h"

int main(int argc, char **argv)
{
    // Create the AHRS instance
    RazorAHRS imu;
    imu.init();

    ros::init(argc, argv, "ahrs_node");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<picopter::Imu>("imu_data", 1);

    ros::Rate loop_rate(100);

    // This is a hack to compute rates until I re-write the AHRS code to send rates over i2c
    float pitch_last = 0.0;
    float roll_last = 0.0;
    float yaw_last = 0.0;
    bool check = false;

    while (ros::ok)
    {
        picopter::Imu msg;
        imu.process();
        msg.pitch = imu.pitch;
        msg.roll = imu.roll;
        msg.yaw = imu.yaw;
        msg.pitch_rate = 0.0;
        msg.roll_rate = 0.0;
        msg.yaw_rate = 0.0;

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}