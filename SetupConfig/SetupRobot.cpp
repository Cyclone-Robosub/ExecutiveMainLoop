#include <filesystem>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
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
}
  
  private:
};