
#include <ros/ros.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <sensor_msgs/JointState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.10", 8007)
    {
        udp.InitCmdData(cmd);
    }
    void UDPUpdate();
    void RobotControl();


    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void Custom::RobotControl()
{
    // initialize node to store joint states and set it as publisher
    static ros::NodeHandle node;
    static ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -5.0f;
    // cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 100){
    if (motiontime >= 0)
    {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if (motiontime >= 0 && motiontime < 10)
        {
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if (motiontime >= 10 && motiontime < 400)
        {
            rate_count++;
            double rate = rate_count / 200.0; // needs count to 200
            // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
            // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            Kp[0] = 20.0;
            Kp[1] = 20.0;
            Kp[2] = 20.0;
            Kd[0] = 2.0;
            Kd[1] = 2.0;
            Kd[2] = 2.0;

            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        float freq_Hz = 1;
        // float freq_Hz = 5;
        float freq_rad = freq_Hz * 2 * M_PI;
        float t = dt * sin_count;
        if (motiontime >= 400)
        {
            sin_count++;
            // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
            sin_joint1 = 0.6 * sin(t * freq_rad);
            sin_joint2 = -0.9 * sin(t * freq_rad);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
        }

        cmd.motorCmd[FR_0].q = qDes[0];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp[0];
        cmd.motorCmd[FR_0].Kd = Kd[0];
        cmd.motorCmd[FR_0].tau = -4.0f;

        cmd.motorCmd[FR_1].q = qDes[1];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp[1];
        cmd.motorCmd[FR_1].Kd = Kd[1];
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q = qDes[2];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp[2];
        cmd.motorCmd[FR_2].Kd = Kd[2];
        cmd.motorCmd[FR_2].tau = 0.0f;
        // cmd.motorCmd[FR_2].tau = 2 * sin(t*freq_rad);
    }

    // if(motiontime > 10){
    //     // safe.PositionLimit(cmd);
    //     safe.PowerProtect(cmd, state, 1);
    //     // safe.PositionProtect(cmd, state, 0.087);
    // }

    // Assigning names and variable the joint state node name and position
    const string leg_names[4] = {"FR", "FL", "RR", "RL"};

    for (uint8_t i = 0; i < 4; i++) {
      if (i == 0) {
        // if current leg is FR
        joint_state.name[i*3] = leg_names[i] + "_hip_joint";
        joint_state.position[i*3] = cmd.motorCmd[FR_0].q;

        joint_state.name[i*3+1] = leg_names[i] + "_thigh_joint";
        joint_state.position[i*3+1] = cmd.motorCmd[FR_1].q;

        joint_state.name[i*3+2] = leg_names[i] + "_calf_joint";
        joint_state.position[i*3+2] = cmd.motorCmd[FR_2].q;
      }else{
        joint_state.name[i*3] = leg_names[i] + "_hip_joint";
        joint_state.position[i*3] = state.motorState[i*3].q;

        joint_state.name[i*3+1] = leg_names[i] + "_thigh_joint";
        joint_state.position[i*3+1] = state.motorState[i*3+1].q;

        joint_state.name[i*3+2] = leg_names[i] + "_calf_joint";
        joint_state.position[i*3+2] = state.motorState[i*3+2].q;
      }
      
    }

    //publish the joint state to ROS
    joint_pub.publish(joint_state);

    // udp.SetSend(cmd);
    
}

int main(int argc, char** argv)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "b1_tf_publisher");
    ros::NodeHandle node;

    Custom custom(LOWLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udp("udp_update", custom.dt, 3, boost::bind(&Custom::UDPUpdate, &custom));

    loop_control.start();
    loop_udp.start();

    ros::spin();

    return 0;
}
