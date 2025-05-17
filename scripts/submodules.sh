cd ../ 
git submodule update --init --recursive --remote
git submodule sync
git submodule status

cd lib/Executive_Propulsion
git checkout master

cd ../Research
git checkout main

cd ../gtest
git checkout main

cd ../yaml-cpp
git checkout master

cd ../../ros2_ws/src/inertial-sense-sdk
git checkout main

cd src/libusb
git checkout master

cd ../../../ros2
git checkout main

cd ../../../
git submodule foreach --recursive git pull
