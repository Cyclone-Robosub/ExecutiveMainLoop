#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
trap killgroup SIGINT

killgroup(){
  kill 0
}

cd ../
source /opt/ros/jazzy/setup.bash

source install/setup.bash # in order to source it for the executive build
colcon build --cmake-args -DMOCK_RPI=OFF &

chmod +x startup.sh

wait