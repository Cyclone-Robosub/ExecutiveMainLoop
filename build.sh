#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd ros2_ws/src
colcon build
. install/setup.bash./
cd ../../

colcon build 




