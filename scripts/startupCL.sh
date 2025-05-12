trap killgroup SIGINT

killgroup(){
  kill 0
}

cd ../
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 -i Python_CL_Tool.py

wait