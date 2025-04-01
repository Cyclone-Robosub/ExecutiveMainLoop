#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd ros2_ws/src/inertial-sense-sdk/ROS/ros2
colcon build
cd build/inertial_sense_ros2
./inertial_sense_ros2_node &
cd ../../../../../../..

colcon build
ros2 run executive_main_loop ExecutiveExecutable 



