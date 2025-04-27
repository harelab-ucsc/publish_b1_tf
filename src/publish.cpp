
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"


using namespace UNITREE_LEGGED_SDK;


class Custom
{
public:
    Custom(std::string parent_frame_id, std::string child_frame_id):
        udp(HIGHLEVEL, 8090, "192.168.123.220", 8082)
    {
        // udp.print = true;
        this->parent_frame = parent_frame_id;
        this->child_frame = child_frame_id;
    }
    void UDPUpdate();
    void RobotControl();
    void publishTF(std::array<float, 3> pos, std::array<float, 3> rpy);

    UDP udp;
    HighState state = {0};
    float dt = 0.002; // 0.001~0.01
    std::string parent_frame;
    std::string child_frame;
};


void Custom::UDPUpdate()
{
    udp.Recv();
    udp.Send();
}


void Custom::RobotControl()
{
    udp.GetRecv(state);

    publishTF(state.position, state.imu.rpy);
}

void Custom::publishTF(std::array<float, 3> pos, std::array<float, 3> rpy)
{
  static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
  
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = parent_frame;
	transformStamped.child_frame_id = child_frame;
	transformStamped.transform.translation.x = pos[0];
	transformStamped.transform.translation.y = pos[1];
	transformStamped.transform.translation.z = pos[2];
	tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);   
}

int main(int argc, char** argv){
  ros::init(argc, argv, "b1_tf_publisher");
    
  ros::NodeHandle node;

  std::string parent_frame;
  std::string child_frame;
  node.param<std::string>("parent_frame", parent_frame, "map");
  node.param<std::string>("child_frame", child_frame, "base");

  Custom custom = Custom(parent_frame, child_frame);
  InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udp("udp_update", custom.dt, 3, boost::bind(&Custom::UDPUpdate, &custom));

  loop_control.start();
  loop_udp.start();

  ros::spin();
  return 0;
};
