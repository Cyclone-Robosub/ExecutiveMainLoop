#!/bin/bash
# Change directory to src
# Build the package
colcon build

# Source ROS setup files
source /opt/ros/jazzy/setup.bash
source install/setup.bash
chmod +x runIMUSensors

python3 StartRobot.py

ros2 run executive_main_loop ExecutiveLoop
cd ros2_ws
ros2 run inertial_sense_ros2 new_target /src/ros2/launch/example_params.yaml
