#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
source /opt/ros/jazzy/setup.bash

cd ros2_ws/src 
colcon build &
. install/setup.bash
cd ../../

source install/setup.bash # in order to source it for the executive build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug &

chmod +x build.sh 
chmod +x startup.sh




