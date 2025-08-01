trap killgroup SIGINT

killgroup(){
  kill 0
}

cd ../
source /opt/ros/jazzy/setup.bash
source install/setup.bash

#cd lib/Research
#./startuppy.sh &
#cd ../../

cd build/executive_main_loop 
./ExecutiveExecutable > output.txt &
cd ../..

wait