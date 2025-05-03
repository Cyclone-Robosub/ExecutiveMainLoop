#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ExecutiveLoop.h"
#include "SensorsDataConfig.h"
#include "SetupConfig/SetupRobot.cpp"

namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Setup Robot during initialization
  SetupRobot initStateandConfig = SetupRobot();
  
  // Setting a stop set in the beginning of startup
  pwm_array zero_set_array;
  for (int i = 0; i < 8; i++) {
    zero_set_array.pwm_signals[i] = 1500;
  }
  
  std::pair<pwm_array, std::chrono::milliseconds> zero_set_pair(
      zero_set_array, std::chrono::milliseconds(99999999));
  
  std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>> currentPWMandDuration_ptr =
      std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(zero_set_pair);
  
  // State file creation or appending
  std::ofstream stateFile;
  fs::path currentPath = fs::current_path();
  fs::path stateFilePath = currentPath.parent_path().parent_path();
  std::string stateFileString = std::string(stateFilePath) + "/state.csv";
  std::cout << stateFileString << std::endl;
  
  if (!std::filesystem::exists(stateFileString)) {
    stateFile.open(stateFileString, std::ofstream::app);
    // Append this for every new file.
    stateFile << "Time,Depth(m),Pressure, IMU Data, PWM Data" << std::endl;
    std::cout << "Created new state file." << std::endl;
  } else {
    stateFile.open(stateFileString, std::ofstream::app);
    std::cout << "Appending to current state file" << std::endl;
  }
  
  // Setup Pins
  auto PhysicalPins = std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<PwmPin *> thrusterPins;
  
  for (auto i : PhysicalPins) {
    thrusterPins.push_back(new HardwarePwmPin(i));
  }
  
  // Setup the Command Interpreter's pins with the physical pins
  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr = 
      std::make_unique<Command_Interpreter_RPi5>(thrusterPins, std::vector<DigitalPin *>{});
  commandInterpreter_ptr->initializePins();
  
  // Create ExecutiveLoop object
  std::cout << "Executive Main Loop Object Creation" << std::endl;
  std::shared_ptr<ExecutiveLoop> mainLoopObject =
      std::make_shared<ExecutiveLoop>(std::move(commandInterpreter_ptr), 
                                      currentPWMandDuration_ptr, 
                                      stateFile, 
                                      std::cout, 
                                      std::cerr);
  
  // Create SensorsDataConfig object
  std::shared_ptr<SensorsDataConfig> sensorsROScallback =
      std::make_shared<SensorsDataConfig>(mainLoopObject);
  
  // Create threads for main loop functions
  std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);
  std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop, mainLoopObject);
  std::jthread SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand, mainLoopObject);
  
  std::cout << "User defined threads has ran successfully" << std::endl;
  
  // Create ROS2 executor
  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
  SensorsExecutor.add_node(sensorsROScallback);
  
  std::cout << "ROS2 running" << std::endl;
  SensorsExecutor.spin();
  
  std::cout << "ROS2 exited." << std::endl;
  rclcpp::shutdown();
  
  return 0;
}
