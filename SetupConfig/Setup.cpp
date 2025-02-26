#include <filesystem>
#include <iostream>
#include <string>
namespace fs = std::filesystem;
class Setup {
public:
    //Reads the Init.yaml and state.yaml
  Setup() {
    if (fs::exists("Init.yaml")) {
    } else {
      std::cerr << "Cannot find Init.yaml file in desired location"
                << std::endl;
      throw std::runtime_error(
          "Cannot find Init.yaml file in desired location");
    }
    
  }
  private:
};