# DriverEnvironment


## Set up
Run only once:
```
cd misc
chmod +x ros2jazzyinstall.sh
./ros2jazzyinstall.sh
```
These are the commands the above script runs.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt update && sudo apt upgrade
sudo apt install build-essentials cmake ninja-build ros-jazzy-desktop
sudo apt-get install gdb
```
Ninja is optional but is faster then makefiles

Once you have clone the repo:
```
cd ExecutiveMainLoop
git submodule update --init --recursive --remote
```
If you have never build ros at least once:
```
cd ros2_ws/src
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build
cd ..
. install/setup.bash
colcon build
```
the build need to be repeated for the setup.bash srcipt to generate. Otherwise build has missing env vars.

## build
Re run the colcon build (this time only once):
```
cd ros2_ws/src
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd ../../
mkdir build && cd build
cmake -G "Ninja" .. && make
```
replace the last line with `cmake .. && make` if you didn't install ninja

If you encounter the folowing:
```
CMake Error: Error: generator Unix Makefiles
does not match with the genrator used prevously: Ninja
```
or vice versa, got to the root of the project, then:
```
rm -rf build/*
```
Then rerun the build command.