
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
    Custom() : safe(LeggedType::B1),
               udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
    {
        udp.InitCmdData(cmd);
    }
    void UDPUpdate();
    void RobotControl();
    void ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg);

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    float joint_positions[12] = {0};

    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}

void Custom::ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg)
{
    for (size_t i = 0; i < guiMsg->position.size() && i < 12; ++i) {
        joint_positions[i] = guiMsg->position[i];  // Store the joint position from the GUI
    }
 }

void Custom::RobotControl()
{
    for (uint8_t i = 0; i < 12; i++) {
        cmd.motorCmd[i].mode = 0; // FOC mode?
        cmd.motorCmd[i].q = joint_positions[i];  // * gear ratio 8.66
        cmd.motorCmd[i].Kp = 20.0;
        cmd.motorCmd[i].Kd = 2.0;
        cmd.motorCmd[i].dq = 0.0;
        cmd.motorCmd[i].tau = 0.0;
    }
    safe.PositionLimit(cmd);
    ROS_WARN("Joint 4 pos: %f\n", cmd.motorCmd[3].q);
    udp.SetSend(cmd);  // Send the motor commands via UDP
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "b1_command_publisher");
    ros::NodeHandle node;
    ROS_INFO("Running RobotControl...");

    Custom custom;
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udp("udp_update", custom.dt, 3, boost::bind(&Custom::UDPUpdate, &custom));

    // message queue of 1
    ros::Subscriber sub = node.subscribe("joint_states", 1, &Custom::ReadSliderGUI, &custom);
    
    loop_control.start();
    loop_udp.start();

    ros::spin();

    return 0;
}
