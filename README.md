# DriverEnvironment


## Set up
Run only once to set up ROS on your environment:
```
cd misc
chmod +x ros2jazzyinstall.sh
./ros2jazzyinstall.sh
```
Note : These are the general commands the above script runs.
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
Run from /ExecutiveMainLoop
```
chmod +x build.sh
./build.sh
```
## Run the robot
again run from /ExecutiveMainLoop
```
chmod +x startup.sh
./startup.sh
```
Note: In the Robot/Rasberry Pi, the startup.sh should run on startup, and the build files should already be built on the Pi 5.

Tips
---
If you encounter the following:
```
CMake Error: Error: generator Unix Makefiles
does not match with the generator used previously: Ninja
```
or vice versa, got to the root of the project, then:
```
rm -rf build/*
```
Then, rerun the build command.

```
WARNING:colcon.colcon_cmake.task.cmake.build:Could not run installation step for package 'InertialSenseSDK' because it has no 'install' target
```
The above error does not affect the final executable.

Non-existing Submodule Paths

If you encounter the following or something similar:
```
git submodule update --init --recursive --remote
fatal: No url found for submodule path 'lib/Propulsion' in .gitmodules
```
or any other submodule, do:
```
git rm lib/Propulsion
```
or the submodule path in question.
Then do:
```
git submodule sync
```
And finally, try initing the submodules.
