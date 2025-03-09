#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build

ros2 run executive_main_loop ExecutiveLoop
# ros2 run inertial_sense_ros2 new_target ../../params/ExecutiveMainLoop_params.yaml

