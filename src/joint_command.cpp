
#include <ros/ros.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <sensor_msgs/JointState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <QApplication>
#include <QWidget>
#include <QCheckBox>
#include <QVBoxLayout>

using namespace std;
using namespace UNITREE_LEGGED_SDK;


class CheckboxGui : public QWidget {

public:
    CheckboxGui() {
        setWindowTitle("Select Joints");
        setGeometry(100, 100, 300, 400);

        QVBoxLayout* layout = new QVBoxLayout(this);

        const QString leg_names[4] = {"FR", "FL", "RR", "RL"};

        for (uint8_t i = 0; i < 4; i++) {
            QCheckBox* checkbox = new QCheckBox(leg_names[i] + "_hip_joint", this);
            checkboxes_.push_back(checkbox);
            layout->addWidget(checkbox);

            QCheckBox* checkbox_thigh = new QCheckBox(leg_names[i] + "_thigh_joint", this);
            checkboxes_.push_back(checkbox_thigh);
            layout->addWidget(checkbox_thigh);

            QCheckBox* checkbox_calf = new QCheckBox(leg_names[i] + "_calf_joint", this);
            checkboxes_.push_back(checkbox_calf);
            layout->addWidget(checkbox_calf);
        }
    }

    bool getCheckboxStates(int i) {
        return checkboxes_[i]->isChecked();
    }

private:
    std::vector<QCheckBox*> checkboxes_;
};



class B1Interface
{
public:
    B1Interface(boost::function<bool(int)> getEnabledMotors): 
        safe(LeggedType::B1),
        udp(LOWLEVEL, 8090, "192.168.123.10", 8007)
    {
        this->getEnabledMotors = getEnabledMotors;
        udp.InitCmdData(cmd);
    }
    void UDPUpdate();
    void RobotControl();
    void ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg);

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    boost::function<bool(int)> getEnabledMotors;
    float joint_positions[12] = {0};

    float dt = 0.002; // 0.001~0.01
};

void B1Interface::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}

void B1Interface::ReadSliderGUI(const sensor_msgs::JointState::ConstPtr& guiMsg)
{
    for (size_t i = 0; i < guiMsg->position.size() && i < 12; ++i) {
        joint_positions[i] = guiMsg->position[i];  // Store the joint position from the GUI
    }
}

void B1Interface::RobotControl()
{
    udp.GetRecv(state);

    // publish joints
    static ros::NodeHandle node;
    static ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_readings", 1);

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(12);
    joint_state.position.resize(12);

    const string leg_names[4] = {"FR", "FL", "RR", "RL"};

    for (uint8_t i = 0; i < 4; i++) {
      joint_state.name[i*3] = leg_names[i] + "_hip_joint";
      joint_state.position[i*3] = state.motorState[i*3].q;

      joint_state.name[i*3+1] = leg_names[i] + "_thigh_joint";
      joint_state.position[i*3+1] = state.motorState[i*3+1].q;

      joint_state.name[i*3+2] = leg_names[i] + "_calf_joint";
      joint_state.position[i*3+2] = state.motorState[i*3+2].q;
    }

    //send the joint state
    joint_pub.publish(joint_state);

    for (uint8_t i = 0; i < 12; i++) {
        if (getEnabledMotors(i)) {
            ROS_WARN("MOTOR %d ENABLED: %f\n", i, joint_positions[i]);
            cmd.motorCmd[i].mode = 0; // FOC mode?
            cmd.motorCmd[i].q = joint_positions[i];  // * gear ratio 8.66
            cmd.motorCmd[i].Kp = 20.0;
            cmd.motorCmd[i].Kd = 2.0;
            cmd.motorCmd[i].dq = 0.0;
            cmd.motorCmd[i].tau = 0.0;
        } else {
            cmd.motorCmd[i] = {0};
        }
    }
    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 1); // 1 is 10% power limit, 10 is 100%
    // udp.SetSend(cmd);  // Send the motor commands via UDP
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "b1_command_publisher");
    ros::NodeHandle node;
    ROS_INFO("Running RobotControl...");

    // GUI stuff
    QApplication app(argc, argv);
    CheckboxGui window;
    window.show();

    auto checkBoxStateFunc = boost::bind(&CheckboxGui::getCheckboxStates, &window, _1);

    // UDP comms stuff
    B1Interface interface(checkBoxStateFunc);
    InitEnvironment();
    LoopFunc loop_control("control_loop", interface.dt, boost::bind(&B1Interface::RobotControl, &interface));
    LoopFunc loop_udp("udp_update", interface.dt, 3, boost::bind(&B1Interface::UDPUpdate, &interface));

    // message queue of 1
    ros::Subscriber sub = node.subscribe("joint_states", 1, &B1Interface::ReadSliderGUI, &interface);
    
    loop_control.start();
    loop_udp.start();
    
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        app.processEvents();
        loop_rate.sleep();
    }

    return 0;
}
