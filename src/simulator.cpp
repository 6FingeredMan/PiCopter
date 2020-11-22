/*
* @file      Simulator.cpp
* @date      11/21/2020
* @copyright Brendan Martin
* @version   1.0.0
* @brief     Defines the Simulator Class
*/

// Included Libraries
#include <cstdint>
#include <map>
#include <math.h>

// 3rd Party Libraries
#include "INIReader.h"
#include <ros/ros.h>

// User Libraries
#include "Controllers.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"
#include "simulator.h"

Simulator::Simulator():
    M1_cmd(0.0),
    M2_cmd(0.0),
    M3_cmd(0.0),
    M4_cmd(0.0),
    M1_RPM(0.0),
    M2_RPM(0.0),
    M3_RPM(0.0),
    M4_RPM(0.0),
    M1_F(0.0),
    M2_F(0.0),
    M3_F(0.0),
    M4_F(0.0),
    M1_T(0.0),
    M2_T(0.0),
    M3_T(0.0),
    M4_T(0.0),
    u_dot(0.0),
    v_dot(0.0),
    w_dot(0.0),
    u(0.0),
    v(0.0),
    w(0.0),
    p_dot(0.0),
    q_dot(0.0),
    r_dot(0.0),
    p(0.0),
    q(0.0),
    r(0.0),
    phi_dot(0.0),
    theta_dot(0.0),
    psi_dot(0.0),
    phi(0.0),
    theta(0.0),
    psi(0.0),
    X_dot(0.0),
    Y_dot(0.0),
    Z_dot(0.0),
    X(0.0),
    Y(0.0),
    Z(0.0),
    K(0.0),
    M(0.0),
    N(0.0),
    dt(0.01),
    reader("/root/ros_catkin_ws/src/picopter/config/simulation.ini")
{
    loadConfig();
    startSimulator();
}

void Simulator::loadConfig(void)
{
    // Get Robot Parameters
    mass = reader.GetReal("Robot", "mass", -1);
    Ixx = reader.GetReal("Robot", "Ixx", -1);
    Iyy = reader.GetReal("Robot", "Iyy", -1);
    Izz = reader.GetReal("Robot", "Izz", -1);
    LA = reader.GetReal("Robot", "Lever Arm", -1);
    Km = reader.GetReal("Robot", "Km", -1);
    Tm = reader.GetReal("Robot", "Tm", -1);
    Voltage = reader.GetReal("Robot", "Voltage", -1);

    // Get Environment Parameters
    g = reader.GetReal("Environment", "gravity", -1);
    rho = reader.GetReal("Environment", "rho", -1);
    lat = reader.GetReal("Enviornment", "latitude", -1);
    lon = reader.GetReal("Environment", "longitude", -1);
}

void Simulator::startSimulator(void)
{
    // First, fire up the subscriber to the motor commands
    motors_sub = n.subscribe<picopter::Motors_msg>("motor_cmds", 1, &Simulator::setMotorCmd, this);
    ros::Rate loop_rate(100);
    while (ros::ok)
    {
        process();

        // The simulator is running at 100 Hz, but other sensor data
        // is being published at a slower rate, so we need to check
        // whether it is time to publish that data
    }
}

void Simulator::rungeKutta(void)
{
    // TO DO -Add a runge-kutta 4th order solver instead of 
    // direct integration

}

void Simulator::process(void)
{
    // Calculate the body relative forces - mechanics
    X = 0.0;
    Y = 0.0;
    Z = M1_F + M2_F + M3_F + M4_F;
    // Calculate the body relative forces - add aerodynamics

    // Calculate the body relative torques
    K = (M1_F + M4_F - M2_F - M3_F) * LA;
    M = (M1_F + M2_F - M3_F - M4_F) * LA;
    N = (M1_T + M2_T + M3_T + M4_T);
    // Calculate the body relative torques - add aerodynamics

    // Calculate Accelerations
    calcAccels();
    calcVelocities();
    calcPositions();
}

void Simulator::calcAccels(void)
{
    u_dot = -g*sin(theta) + r*v - q*w;
    v_dot = g*sin(phi)*cos(theta) - r*u + p*w;
    w_dot = (1/mass)*(-Z) + g*cos(phi)*cos(theta) + q*u - p*v;
    p_dot = (1/Ixx)*(K + (Iyy-Izz)*q*r);
    q_dot = (1/Iyy)*(M + (Izz-Ixx)*p*r);
    r_dot = (1/Izz)*(N + (Ixx-Iyy)*p*q);

}

void Simulator::calcVelocities(void)
{
    u = u_dot*dt;
    v = v_dot*dt;
    w = w_dot*dt;
    p = p_dot*dt;
    q = q_dot*dt;
    r = r_dot*dt;
    phi_dot = p + (q*sin(phi) + r*cos(phi))*tan(theta);
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot = (q*sin(phi) + r*cos(phi))*(1/cos(theta));
    X_dot = cos(theta)*cos(psi)*u +
            (-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi))*v +
            (sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi))*w;
    Y_dot = cos(theta)*sin(psi)*u + 
            (cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi))*v +
            (-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi))*w;
    Z_dot = -1.0*(-sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w);
}

void Simulator::calcPositions(void)
{
    phi = phi_dot*dt;
    theta = theta_dot*dt;
    psi = psi_dot*dt;
    northings = X_dot*dt;
    eastings = Y_dot*dt;
    elevation = Z_dot*dt;
    
    // Correct for ground
    if(elevation < 0)
    {
        X_dot = 0.0;
        Y_dot = 0.0;
        Z_dot = 0.0;
    }
}

void Simulator::setMotorCmd(const picopter::Motors_msg::ConstPtr& msg)
{
    M1_cmd = msg->M1;
    M2_cmd = msg->M2;
    M3_cmd = msg->M3;
    M4_cmd = msg->M4;
    computeRPM();
    propThrust();
    propTorque();
}

void Simulator::computeRPM(void)
{
    // Convert command to RPM value (instantaneous right now..)
    M1_RPM = M1_cmd*0.9*Voltage;
    M2_RPM = M2_cmd*0.9*Voltage;
    M3_RPM = M3_cmd*0.9*Voltage;
    M4_RPM = M4_cmd*0.9*Voltage;
}

void Simulator::propThrust(void)
{
    // Calculate Prop Thrust, Newtons
    M1_F = M1_RPM*M1_RPM*Km;
    M2_F = M2_RPM*M2_RPM*Km;
    M3_F = M3_RPM*M3_RPM*Km;
    M4_F = M4_RPM*M4_RPM*Km;
}

void Simulator::propTorque(void)
{
    // Calculate Induced Yaw Torque, Newton-meters
    M1_T = -1.0 * (M1_RPM * M1_RPM * Tm);   // CW Prop,  (-1) Torque
    M2_T = (M2_RPM * M2_RPM * Tm);          // CCW Prop, (+1) Torque
    M3_T = -1.0 * (M3_RPM * M3_RPM * Tm);   // CW Prop,  (-1) Torque
    M4_T = (M4_RPM * M4_RPM * Tm);          // CCW Prop, (+1) Torque
}

void Simulator::gpsSim(void)
{
    // TO DO
}

void Simulator::ahrsSim(void)
{
    // TO DO
}

void Simulator::altitudeSim(void)
{
    // TO DO
}

void Simulator::elevationSim(void)
{
    // TO DO
}

