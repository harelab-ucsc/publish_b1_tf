/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    /*
    // Low-level port.
    Custom(uint8_t level): 
        udp(level, 8090, "192.168.123.10", 8007){
    }
    */
    // High-level port.
    Custom(uint8_t level): 
        udp(level, 8090, "192.168.123.220", 8082){
    }
    void UDPUpdate();
    void RobotControl();

    void UDPSend();
    void UDPRecv();

    UDP udp;
    //LowState state = {0};
    HighState state = {0};
    float dt = 0.002;     // 0.001~0.01

};

void Custom::UDPUpdate()
{ 
    udp.Recv();
    udp.Send();
}

void Custom::RobotControl() 
{
    // Timestamp.
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // Receive state data.
    udp.GetRecv(state);

    // Output all data as a CSV to stdout.
    std::cout << timestamp << ",";
    for (uint8_t i = 0; i < 4; i++) {
	std::cout << state.motorState[i].tauEst << ",";
	std::cout << state.motorState[i+1].tauEst << ",";
	std::cout << state.motorState[i+2].tauEst << ",";
    }
    std::cout << state.imu.accelerometer[0] << ",";
    std::cout 	    << state.imu.accelerometer[1] << ",";
    std::cout 	    << state.imu.accelerometer[2] << ",";
    std::cout 	    << state.imu.gyroscope[0] << ",";
    std::cout 	    << state.imu.gyroscope[1] << ",";
    std::cout 	    << state.imu.gyroscope[2];

    std::cout << std::endl;

    /*
    motiontime++;
    udp.GetRecv(state);
    printf("%d  %f  %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq);
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -5.0f;
    // cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    // float freq_Hz = 1;
    // float freq_Hz = 2;
    // float freq_Hz = 5;
    // float freq_rad = freq_Hz * 2* M_PI;
    // float t = dt*sin_count;

    if( motiontime >= 500){
        sin_count++;
        float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
        // float torque = (0 - state.motorState[FR_1].q)*20.0f + (0 - state.motorState[FR_1].dq)*2.0f;
        // float torque = (0 - state.motorState[FR_1].q)*40.0f + (0 - state.motorState[FR_1].dq)*2.0f;
        // float torque = 2 * sin(t*freq_rad);
        if(torque > 5.0f) torque = 5.0f;
        if(torque < -5.0f) torque = -5.0f;
        // if(torque > 15.0f) torque = 15.0f;
        // if(torque < -15.0f) torque = -15.0f;

        // cmd.motorCmd[FR_2].q = PosStopF;
        // cmd.motorCmd[FR_2].dq = VelStopF;
        // cmd.motorCmd[FR_2].Kp = 0;
        // cmd.motorCmd[FR_2].Kd = 0;
        // cmd.motorCmd[FR_2].tau = torque;

        cmd.motorCmd[FR_1].q = PosStopF;
        cmd.motorCmd[FR_1].dq = VelStopF;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 0;
        cmd.motorCmd[FR_1].tau = torque;

    }
    // int res = safe.PowerProtect(cmd, state, 1);
    // if(res < 0) exit(-1);

    udp.SetSend(cmd);
    */
    //sleep(0.01);
}

int main(void)
{
    //std::cout << "Communication level is set to LOW-level." << std::endl
    //          << "WARNING: Make sure the robot is hung up." << std::endl
    //         << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();
    // Print headers.
    std::cout << "Timestamp,FRHipT,FLHipT,RRHipT,RLHipT,FRThighT,FLThighT,RRThighT,RLThighT,FRCalfT,FLCalfT,RRCalfT,RLCalfT,IMUAccx,IMUAccy,IMUAccz,IMUGyrroll,IMUGyrpitch,IMUGyryaw\n";

    //Custom custom(LOWLEVEL);
    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udp("udp_update",     custom.dt, 3, boost::bind(&Custom::UDPUpdate,      &custom));

    loop_control.start();
    loop_udp.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
