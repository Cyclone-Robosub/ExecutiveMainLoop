# Introduction
This was a framework for the main software architecture for the robot during the 2024-2025 season. The idea was to scale all of the robot's functions and tasks with threads based on the Ubuntu operating system to with a primary focus on performance, data synchronization, and efficiency (Threads are easier to switch than processes),
while not giving up completely on readability, flexibility, and testing. 
Setting up, building, and compiling may be outdated due to outdated hardware, such as the IMU. This has been replaced by the leadership's decision for Waypoint Executive and Thrust Control.  

Main contributors: Tanishq Dwivedi, William Barber, and Danny Kwong.

## Set up
---
Run only once to set up ROS on your environment:
```
cd misc
chmod +x ros2jazzyinstall.sh
./ros2jazzyinstall.sh
```
Ninja is optional but is faster then makefiles

Once you have clone the repo:

Check for ros2 symlink inside of ros2_ws/src (Check if files are there)

Check for Executive_Propulsion/Pi/CmakeLists.txt compile problem fixed.

## Build/Compile
---
Run from /ExecutiveMainLoop/scripts
```
chmod +x build.sh
./build.sh
```
or if you are plugged to a nuclear reactor and have 16 cores of gaming 
Make sure you Control-C once finished compling to exit out of the bash script 
for this option.
```
chmod +x buildQuick.sh
./buildQuick.sh
```
## Run the ExecutiveMainLoop Functions
again run from /ExecutiveMainLoop/scripts
```
chmod +x startup.sh
./startup.sh
```
### Manual Control/Python CLTool
from /ExecutiveMainLoop/scripts
```
chmod +x startupCL.sh
./startupCL.sh
```
follow the directions printed or read the code (lol what documentation?)

Note: In the Robot/Rasberry Pi, the startup.sh should run on startup, and the build files should already be built on the Pi 5.

## Testing
---
building for test:
```
colcon build --cmake-args -DBUILD_TESTING=ON
```
Run the test:
```
colcon test --packages-select executive_main_loop
```
Run with detailed test results:
```
colcon test --packages-select executive_main_loop --event-handlers console_direct+
```
To check test results:
```
colcon test-result --all
```
To see which test cases failed (if any):
```
colcon test-result --all --verbose
```
## Tips
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




NOTES:
