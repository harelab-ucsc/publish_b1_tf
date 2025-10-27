
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
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
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

    // Assigning names and variable the joint state node name and position
    const string leg_names[4] = {"FR", "FL", "RR", "RL"};

    static int pos = 1;

    for (uint8_t i = 0; i < 4; i++) {
        // if current leg is FR
        joint_state.name[i*3] = leg_names[i] + "_hip_joint";
        joint_state.position[i*3] = pos;

        joint_state.name[i*3+1] = leg_names[i] + "_thigh_joint";
        joint_state.position[i*3+1] =  pos;

        joint_state.name[i*3+2] = leg_names[i] + "_calf_joint";
        joint_state.position[i*3+2] = pos;

    }
        
    pos = !pos;

    //publish the joint state to ROS
    joint_pub.publish(joint_state);

    // udp.SetSend(cmd);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "b1_command_publisher");
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
