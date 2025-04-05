#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd ros2_ws/src
colcon build
. install/setup.bash
cd build/inertial_sense_ros2
./inertial_sense_ros2_node&
cd ../../../../

colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 run--prefix 'gdbserver localhost:3000' executive_main_loop ExecutiveExecutable 



