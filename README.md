# Publish B1 Transforms

Creates a ROS node that connects the Unitree B1 and uses the high level control mode to read the current reported position of the B1 and publish it as a ROS tf2 transform on the transforms topic. 


# Setup

Follow the directions from here: [link](https://github.com/unitreerobotics/unitree_ros_to_real/tree/B1?tab=readme-ov-file), more details below:
1. Create a catkin workspace, in the `src` folder, clone the git repo unitree_ros_to_real, linked above, but **on the B1 branch**
   - Note, keep the directory structure of that whole repository. In other words your folder structure should be like catkin_ws/src/unitree_ros_to_real/unitree_legged_real/ and unitree_ros_to_real should also have unitree_legged_msgs inside of it. Don't worry about unitree_legged_sdk until next step.
2. replace the unitree_legged_sdk with the version that is on the B1 branch. Either delete the folder and download the right branch and replace it, or `git checkout` that branch after cloning it, etc. Just make sure the folder is still called unitree_legged_sdk
- MAKE SURE TO USE THE B1 BRANCH for BOTH the SDK and the ros_to_real repo
4. check that `catkin_make` works, if not, you've done something wrong. Likely not used the B1 branch for the SDK or the ros_to_real or both. Or you messed with the directory structure.
5. Clone this repo into the src folder of the catkin workspace.
6. Run `catkin_make` and `source ./devel/setup.bash`, etc. Run the launch file using `roslaunch publish_b1_tf publish.launch` 
