/*
* @file      Simulator.h
* @date      11/21/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Simulator Class
*/
#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

// Included Libraries
#include <cstdint>
#include <map>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "Controllers.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Altitude_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Elevation_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Gps_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"

class Simulator
{
    public:
        // Constructor
        Simulator();

        // Reads the simulator.ini and configures the simulation
        void loadConfig(void);

        // Starts the simulator
        void startSimulator(void);

        void rungeKutta(void);

        void process(void);

        void calcAccels(void);

        void calcVelocities(void);

        void calcPositions(void);

        void setMotorCmd(const picopter::Motors_msg::ConstPtr& msg);

        void computeRPM(void);

        void propThrust(void);

        void propTorque(void);

        void gpsSim(void);

        void ahrsSim(void);

        void altitudeSim(void);

        void elevationSim(void);

        // Motor Commands, %
        float M1_cmd;
        float M2_cmd;
        float M3_cmd;
        float M4_cmd;
        // Propeller RPM, RPM
        float M1_RPM;
        float M2_RPM;
        float M3_RPM;
        float M4_RPM;
        // Motor Forces, Newtons
        float M1_F;
        float M2_F;
        float M3_F;
        float M4_F;
        // Motor/Prop Induced Torques about yaw, N-M
        float M1_T;
        float M2_T;
        float M3_T;
        float M4_T;
        // State variables
        float u_dot;            // Body relative acceleration, x, m/s/s
        float v_dot;            // Body relative acceleration, y, m/s/s
        float w_dot;            // Body relative acceleration, z, m/s/s
        float u;                // Body relative velocity, x, m/s
        float v;                // Body relative velocity, y, m/s
        float w;                // Body relative velocity, z, m/s
        float p_dot;            // Body relative angluar acceleration, roll, rad/s/s
        float q_dot;            // Body relative angluar acceleration, pitch, rad/s/s
        float r_dot;            // Body relative angluar acceleration, yaw, rad/s/s
        float p;                // Body relative angluar velocity, roll, rad/s
        float q;                // Body relative angluar velocity, pitch, rad/s
        float r;                // Body relative angluar velocity, yaw, rad/s
        float phi_dot;          // Earth relative angluar velocity, roll, rad/s
        float theta_dot;        // Earth relative angluar velocity, pitch, rad/s
        float psi_dot;          // Earth relative angluar velocity, yaw, rad/s
        float phi;              // Earth relative euler angle, roll, rad
        float theta;            // Earth relative euler angle, pitch, rad
        float psi;              // Earth relative euler angle, yaw, rad
        float X_dot;            // Earth relative velocity, x, m/s
        float Y_dot;            // Earth relative velocity, y, m/s
        float Z_dot;            // Earth relative velocity, z, m/s
        float northings;        // Earth relative location, m
        float eastings;         // Earth relative location, m
        float elevation;        // Earth relative location, m
        float X;                // Body realtive force, N
        float Y;                // Body realtive force, N
        float Z;                // Body realtive force, N
        float K;                // Body realtive torque, N-m
        float M;                // Body realtive torque, N-m
        float N;                // Body realtive torque, N-m
        // Robot Parameters
        float mass;             // Mass, kg
        float Ixx;              // Moment of Inertia, kg-m^2
        float Iyy;              // Moment of Inertia, kg-m^2
        float Izz;              // Moment of Inertia, kg-m^2        
        float LA;               // Asbolute lever arm from quad motors to cg, m
        float Km;               // Prop Thrust Constant
        float Tm;               // Prop Drag Constant to compute torque
        float Voltage;          // Initial battery voltage
        // Environment Parameters
        float g;                // Gravity, m/s/s
        float rho;              // Air Density, kg/m^3
        float lat;              // Latitude, degree decimal
        float lon;              // Longitude, degree decimal
        // Runge-Kutta Solver Variables and Arrays
        float dt;               // Frequency of solver
        //float k1[6];            // Array of 6 accelerations
        //float k2[6];            // ""
        //float k3[6];            // ""
        //float k4[6];            // ""

        // Simulation ini Reader
        INIReader reader;

        // ROS topic messages
        picopter::Altitude_msg alt_data_msg;
        picopter::Imu_msg imu_data_msg;
        picopter::Gps_msg gps_data_msg;
        picopter::Elevation_msg elev_data_msg;

    private:
        // ROS hooks
        ros::NodeHandle n;
        ros::Publisher elevation_pub;
        ros::Publisher ahrs_pub;
        ros::Publisher altitude_pub;
        ros::Publisher gps_pub;
        ros::Subscriber motors_sub;


};


#endif // __SIMULATOR_H__