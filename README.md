# ROS1 TortoiseBot ROSTest 🐢

## Overview 🌟
Welcome to the automated testing universe of the ROS1 TortoiseBot! This repository houses the mighty rostest implementations tailored for the ROS1 TortoiseBot. Designed to ensure the robustness and reliability of the Action Server /tortoisebot_waypoints, this suite comprises essential tests, including positional accuracy and orientation precision checks.
## Features 🚀
- **Automated Testing**: Automated Testing: Empower your TortoiseBot with automated tests to confirm its operational excellence.
- **ROS2 Integration**: Seamlessly woven into the ROS1 fabric, these tests offer a native testing experience in the ROS ecosystem.

## Installation and Usage 🛠️

### Prerequisites
- ROS1 Noetic 
- rostest library 📚

### Installation
Time to get everything set up! Follow these steps:
```
cd ~/simulation_ws/src
git clone https://github.com/tianyaohu/ros2_tortoisebot_GTest.git
git clone --branch noetic https://github.com/rigbetellabs/tortoisebot.git
```
Build and source work space:
```
#source ros1 if necessary
source /opt/ros/noetic/setup.bash
catkin_make && source devel/setup.bash
```

### Start Gazebo Simulation Simulation
In *Terminal #1*
```
source ~/simulation_ws/devel/setup.bash
#Gazebo may NOT start normally the first time, just ctrl+c and retry, 2nd try should work
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```
*Expected Result*
![Start Gazebo](media/start_gazebo.gif)

## Start Testing🎯
In *Terminal #2*
```
source ~/simulation_ws/devel/setup.bash
# defaults destination is x=0.5, y=0.5
rostest tortoisebot_waypoints integration_test.test x:=-0.5 y:=0.5 --reuse-master
```

Expected result:
![Testing](media/ros1_test.gif)

