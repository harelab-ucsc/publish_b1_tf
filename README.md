# Publish B1 Transforms

Creates a ROS node that connects the Unitree B1 and uses the high level control mode to read the current reported position of the B1 and publish it as a ROS tf2 transform on the transforms topic. 


# Setup

Follow the directions from here: [link](https://github.com/unitreerobotics/unitree_ros_to_real/tree/B1?tab=readme-ov-file), more details below:
1. Create a catkin workspace, in the `src` folder, clone the above git repo but **on the B1 branch**
2. MAKE SURE TO USE THE B1 BRANCH for BOTH the SDK and the ros_to_real repo
3. replace the unitree_legged_sdk with the version that is on the B1 branch
4. check that `catkin_make` works, if not, you've done something wrong
5. Clone this repo into the src folder of the catkin workspace.
6. Run `catkin_make` and `source ./devel/setup.bash`, etc. Run the launch file using `roslaunch publish_b1_tf publish.launch` 
