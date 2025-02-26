#include <filesystem>
#include <iostream>
#include <string>
namespace fs = std::filesystem;
class Setup {
public:
    //Reads the Init.yaml and state.yaml
    //We can create a ROS pacakdge of the Statefile and load in using ROS parameters and not have to anything because we will make a ROS node of the state.yaml file.
  Setup() {
    if (fs::exists("Init.yaml")) {
      //Need to decide how to set the variables
        //Propulsion ->Thruster Controller call?
        // How to make sure everything is setup here or in the main loop
    } else {
      std::cerr << "Cannot find Init.yaml file in desired location"
                << std::endl;
      throw std::runtime_error(
          "Cannot find Init.yaml file in desired location");
    }
    fs::path currentPath = fs::current_path();
    fs::path parentPath = currentPath.parent_path();
    std::string stateDirectory;
    stateDirectory = std::string(parentPath) + "state.yaml";
    if(fs::exists(parentPath)){

    }
    
  }
  private:
};