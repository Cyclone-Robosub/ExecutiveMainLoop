cd ../
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd ros2_ws/src/
. install/setup.bash
cd build/inertial_sense_ros2
./inertial_sense_ros2_node > IMUrosstuff.txt &
cd ../../../../

cd lib/Research
./startuppy.sh &
cd ../../


source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd build/executive_main_loop 
./ExecutiveExecutable > output.txt &
cd ../..


