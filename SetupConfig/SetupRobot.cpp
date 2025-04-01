#include <filesystem>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include "inertial_sense_ros2.cpp"
namespace fs = std::filesystem;
class SetupRobot {
public:
    //Reads the Init.yaml and state.yaml
    //We can create a ROS package of the Statefile and load in using ROS parameters and not have to anything because we will make a ROS node of the state.yaml file.
  SetupRobot() {
    fs::path currentPaths = fs::current_path();
    fs::path parentPaths = currentPaths.parent_path();
    if (std::filesystem::exists(std::string(currentPaths) + "/SetupConfig/InitIMU.yaml")) {
      setupIMUwithROS();
    } else {
      std::cerr << "Cannot find InitIMU.yaml file in desired location"
                << std::endl;
                std::cout << std::string(currentPaths);
    //  throw std::runtime_error(
      //    "Cannot find Init.yaml file in desired location");
    }
    
  }
  void setupIMUwithROS(){
    std::cout << "Found InitIMU.yaml" << std::endl;
    std::string IMUyamlconfig = "topic: \"inertialsense\"\n"
                       "port: [/dev/ttyACM0, /dev/ttyACM1, /dev/ttyACM2]\n"
                       "baudrate: 921600\n"
                       "\n"
                       "ins:\n"
                       "  navigation_dt_ms: 16                          # EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate."
                       "\n"
                       "sensors:\n"
                       "  messages:  \n"
                       "    pimu:             # Publish preintegrated IMU delta theta and delta velocity\n"
                       "      topic: \"pimu\"\n"
                       "      enable: true\n"
                       "      period: 1\n";
      YAML::Node config = YAML::Load(IMUyamlconfig);
      if(config.IsDefined()){
      InertialSenseROS IMUROS(config);
      IMUROS.initalize();
  }  
}
  
  private:
};