#!/bin/bash
# Change directory to src
# Build the package
# Source ROS setup files
trap killgroup SIGINT

killgroup(){
  kill 0
}

chmod +x submodules.sh
./submodules.sh

cd ../
source /opt/ros/jazzy/setup.bash

cd ros2_ws/src 
colcon build &
. install/setup.bash
cd ../../

cd lib/Research/src
colcon build &
cd ../
chmod +x startuppy.sh
cd ../../

source install/setup.bash # in order to source it for the executive build
colcon build --cmake-args -DMOCK_RPI=OFF &

cd scripts
chmod +x startup.sh
cd ../

wait