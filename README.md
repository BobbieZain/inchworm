# inchworm
The robot consists of 2 packages, "description" which contains the robot description and "movement" which contains the control and moveit files. 


The files can be launched using
colcon build
source install/setup.bash
ros2 launch "package name" "launchfile name"
**shortcut:**
make gazebo - colcon builds and launches gazebo.launch.py - launches gazebo fortress and rviz2
