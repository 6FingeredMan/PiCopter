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
#include "/root/ros_catkin_ws/devel/include/picopter/Imu_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Motors_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Navigator_msg.h"
#include "/root/ros_catkin_ws/devel/include/picopter/Sim_msg.h"
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
    northings(0.0),
    eastings(0.0),
    elevation(0.0),
    X(0.0),
    Y(0.0),
    Z(0.0),
    K(0.0),
    M(0.0),
    N(0.0),
    sim_freq(200.0),
    dt(0.005),
    sim_time(0.0),
    reader("/root/ros_catkin_ws/src/picopter/config/simulation.ini")
{
    loadConfig();
    startSimulator();
}

void Simulator::loadConfig(void)
{
    // Get Robot Parameters
    mass = float(reader.GetReal("Robot", "mass", 0.4377));
    Ixx = float(reader.GetReal("Robot", "Ixx", 0.0047));
    Iyy = float(reader.GetReal("Robot", "Iyy", 0.0047));
    Izz = float(reader.GetReal("Robot", "Izz", 0.9998));
    LA = float(reader.GetReal("Robot", "Lever Arm", .125));
    Km = float(reader.GetReal("Robot", "Km", -1));
    Tm = float(reader.GetReal("Robot", "Tm", -1));
    Voltage = float(reader.GetReal("Robot", "Voltage", -1));

    // Get Environment Parameters
    g = float(reader.GetReal("Environment", "gravity", 9.81));
    rho = float(reader.GetReal("Environment", "rho", 1.225));
    lat = float(reader.GetReal("Enviornment", "latitude", -1));
    lon = float(reader.GetReal("Environment", "longitude", -1));

    // Get Simulation Parameters
    sim_freq = double(reader.GetReal("Simulator", "simulation frequency (hz)", 200));
    dt = 1.0 / sim_freq;
}

void Simulator::startSimulator(void)
{
    // First, fire up the subscriber to the motor commands
    motors_sub = n.subscribe<picopter::Motors_msg>("motor_cmds", 1, &Simulator::setMotorCmd, this);

    // Then fire up the simulation loop
    sim_pub = n.advertise<picopter::Sim_msg>("sim_data", 1);
    ros::Rate loop_rate(sim_freq);
    while (ros::ok)
    {
        //ROS_INFO("Looping");
        process();
        sim_time += dt;
        sim_data_msg.pitch = theta;
        sim_data_msg.roll = phi;
        sim_data_msg.yaw = psi;
        sim_data_msg.pitch_rate = theta_dot;
        sim_data_msg.roll_rate = phi_dot;
        sim_data_msg.yaw_rate = psi_dot;
        sim_data_msg.elevation = elevation;
        sim_data_msg.altitude = elevation;
        sim_data_msg.northings = northings;
        sim_data_msg.eastings = eastings;
        sim_data_msg.latitude = 0.0;
        sim_data_msg.longitude = 0.0;
        sim_data_msg.motor_1_force = M1_F;
        sim_data_msg.motor_2_force = M2_F;
        sim_data_msg.motor_3_force = M3_F;
        sim_data_msg.motor_4_force = M4_F;
        sim_data_msg.X_Force = X;
        sim_data_msg.Y_Force = Y;
        sim_data_msg.Z_Force = Z;
        sim_data_msg.K_Moment = K;
        sim_data_msg.M_Moment = M;
        sim_data_msg.N_Moment = N;
        sim_data_msg.u_dot = u_dot;
        sim_data_msg.v_dot = v_dot;
        sim_data_msg.w_dot = w_dot;
        sim_data_msg.p_dot = p_dot;
        sim_data_msg.q_dot = q_dot;
        sim_data_msg.r_dot = r_dot;
        sim_data_msg.u = u;
        sim_data_msg.v = v;
        sim_data_msg.w = w;
        sim_data_msg.p = p;
        sim_data_msg.q = q;
        sim_data_msg.r = r;
        sim_data_msg.sim_time = sim_time;
        sim_pub.publish(sim_data_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Simulator::process(void)
{
    // Check for NaN and reset to values to 0.0
    if (isnan(theta)) theta = 0.0;
    if (isnan(phi)) phi = 0.0;
    if (isnan(psi)) psi = 0.0;

    // Calculate the body relative forces - mechanics
    X = -mass*g*std::sin(theta); // theta
    Y = mass*g*std::sin(phi)*std::cos(theta); // phi & theta
    Z = -M1_F - M2_F - M3_F - M4_F; // Prop forces are negative due to NED reference frame
    Z = Z + mass*g*std::cos(phi)*std::cos(theta);
    // Calculate the body relative forces - add aerodynamics
    // TBD

    // Calculate the body relative torques
    K = (M1_F + M2_F - M3_F - M4_F) * LA;          // Roll
    M = (M1_F + M4_F - M2_F - M3_F) * LA;          // Pitch
    N = (M1_T + M2_T + M3_T + M4_T);               // Yaw
    // Calculate the body relative torques - add aerodynamics

    // Calculate Accelerations
    calcAccels();
    calcVelocities();
    calcPositions();
}

// float * Simulator::rungeKutta(float input_array[12])
// {

// }

void Simulator::calcAccels(void)
{
    // Check for NaN and reset to values to 0.0
    if (isnan(p)) p = 0.0;
    if (isnan(q)) q = 0.0;
    if (isnan(r)) r = 0.0;
    if (isnan(u)) u = 0.0;
    if (isnan(v)) v = 0.0;
    if (isnan(w)) w = 0.0;

    u_dot = -g*std::sin(theta) + r*v - q*w;
    v_dot = g*std::sin(phi)*std::cos(theta) - r*u + p*w;
    w_dot = (1/mass)*(Z) + q*u - p*v;
    p_dot = (1/Ixx)*(K + (Iyy-Izz)*q*r);
    q_dot = (1/Iyy)*(M + (Izz-Ixx)*p*r);
    r_dot = (1/Izz)*(N + (Ixx-Iyy)*p*q);

}

void Simulator::calcVelocities(void)
{

    // Check for NaN and reset to values to 0.0
    if (isnan(p_dot)) p_dot = 0.0;
    if (isnan(q_dot)) q_dot = 0.0;
    if (isnan(r_dot)) r_dot = 0.0;
    if (isnan(u_dot)) u_dot = 0.0;
    if (isnan(v_dot)) v_dot = 0.0;
    if (isnan(w_dot)) w_dot = 0.0;

    // Using direct integration right now
    // First, body translational velocities and rotational velocities
    u += u_dot*dt;
    v += v_dot*dt;
    w += w_dot*dt;
    p += p_dot*dt;
    q += q_dot*dt;
    r += r_dot*dt;

    // Second, Earth relative rotational velocities and translational velocities
    phi_dot = p + (q*std::sin(phi) + r*std::cos(phi))*std::tan(theta);
    theta_dot = q*std::cos(phi) - r*std::sin(phi);
    psi_dot = (q*std::sin(phi) + r*std::cos(phi))*(1/std::cos(theta));
    X_dot = std::cos(theta)*std::cos(psi)*u +
            (-std::cos(phi)*std::sin(psi) + std::sin(phi)*std::sin(theta)*std::cos(psi))*v +
            (std::sin(phi)*std::sin(psi) + std::cos(phi)*std::sin(theta)*std::cos(psi))*w;
    Y_dot = std::cos(theta)*std::sin(psi)*u + 
            (std::cos(phi)*std::cos(psi) + std::sin(phi)*std::sin(theta)*std::sin(psi))*v +
            (-std::sin(phi)*std::cos(psi) + std::cos(phi)*std::sin(theta)*std::sin(psi))*w;
    Z_dot = -1.0*(-std::sin(theta)*u + std::sin(phi)*std::cos(theta)*v + std::cos(phi)*std::cos(theta)*w);
}

void Simulator::calcPositions(void)
{
    // Using direct integration right now
    phi += phi_dot*dt;
    theta += theta_dot*dt;
    psi += psi_dot*dt;
    northings += X_dot*dt;
    eastings += Y_dot*dt;
    elevation += Z_dot*dt;
    
    // Correct for ground
    if(elevation < 0.0)
    {
        elevation = 0.0;
        // If the quad is on the ground, force its velocity to be greater than or equal to zero
        if(Z_dot < 0.0)
        {
            w_dot = 0.0;
            w = 0.0;
            Z_dot = 0.0;
        }
    }
}

void Simulator::setMotorCmd(const picopter::Motors_msg::ConstPtr& msg)
{
    M1_cmd = msg->M1;
    M2_cmd = msg->M2;
    M3_cmd = msg->M3;
    M4_cmd = msg->M4;
    propThrust();
    propTorque();
}

// void Simulator::computeRPM(void)
// {
//     // Convert command to RPM value (instantaneous right now..)
//     M1_RPM = M1_cmd*0.9*Voltage;
//     M2_RPM = M2_cmd*0.9*Voltage;
//     M3_RPM = M3_cmd*0.9*Voltage;
//     M4_RPM = M4_cmd*0.9*Voltage;
// }

void Simulator::propThrust(void)
{
    // Convert throttle command to micro seconds
    float M1_DC = ( M1_cmd * 9.30 + 1070 );
    float M2_DC = ( M2_cmd * 9.30 + 1070 );
    float M3_DC = ( M3_cmd * 9.30 + 1070 );
    float M4_DC = ( M4_cmd * 9.30 + 1070 );

    // Covert throttle in micro seconds to a force in grams
    // using best fit empirical data
    if (M1_DC <= 1070) 
    {
        M1_F = 0.0;
    }
    else
    {
        M1_F = 0.0007 * M1_DC * M1_DC - 0.6577 * M1_DC;
    }

    if (M2_DC <= 1070) 
    {
        M2_F = 0.0;
    }
    else
    {
        M2_F = 0.0007 * M2_DC * M2_DC - 0.6577 * M2_DC;
    }

    if (M3_DC <= 1070) 
    {
        M3_F = 0.0;
    }
    else
    {
        M3_F = 0.0007 * M3_DC * M3_DC - 0.6577 * M3_DC;
    }

    if (M4_DC <= 1070) 
    {
        M4_F = 0.0;
    }
    else
    {
        M4_F = 0.0007 * M4_DC * M4_DC - 0.6577 * M4_DC;
    }

    // Convert Motor forces from grams to Newtons
    M1_F = (M1_F / 1000.0) * 9.81;
    M2_F = (M2_F / 1000.0) * 9.81;
    M3_F = (M3_F / 1000.0) * 9.81;
    M4_F = (M4_F / 1000.0) * 9.81;

}

void Simulator::propTorque(void)
{
    // Calculate Induced Yaw Torque, Newton-meters
    M1_T = 0.0;
    M2_T = 0.0;
    M3_T = 0.0;
    M4_T = 0.0;
    //M1_T = -1.0 * (M1_RPM * M1_RPM * Tm);   // CW Prop,  (-1) Torque
    //M2_T = (M2_RPM * M2_RPM * Tm);          // CCW Prop, (+1) Torque
    //M3_T = -1.0 * (M3_RPM * M3_RPM * Tm);   // CW Prop,  (-1) Torque
    //M4_T = (M4_RPM * M4_RPM * Tm);          // CCW Prop, (+1) Torque
}

