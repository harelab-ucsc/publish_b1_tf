
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
    void ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg);

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

void Custom::ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg)
{
    ROS_INFO("FL first joint: %s = %.3f", guiMsg->name[0].c_str(), guiMsg->position[0]);
}

void Custom::RobotControl()
{
    // initialize node to store joint states and set it as publisher
    static ros::NodeHandle node;
    static ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);

    // vals = sub.read()

    // cmd.motor1 = vals[0];

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

    ros::Subscriber sub = node.subscribe("joint_states", 10, &Custom::ReadSliderGUI, &custom);

    loop_control.start();
    loop_udp.start();

    ros::spin();

    return 0;
}
