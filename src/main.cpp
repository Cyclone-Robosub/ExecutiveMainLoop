#ifndef TESTING_EXCLUDE_MAIN
#include "ExecutiveLoop.hpp"
#include "Pwm_Command.hpp"
#include "SensorDataConfig.hpp"
#include <iostream>
#include <fstream>

std::ofstream makeStateFile(std::ostream& output, std::ostream& error) {
    std::ofstream stateFile;
    fs::path stateFilePath = fs::current_path().parent_path().parent_path();
    std::string stateFileString = std::string(stateFilePath) + "/state.csv";
    output << stateFileString << std::endl;
    if (!std::filesystem::exists(stateFileString)) {
      stateFile.open(stateFileString, std::ofstream::app);
  
      // Append this for every new file.
      stateFile << "Time,Depth(m),Pressure,IMU Data,PWM Data" << std::endl;
      output << "Created new state file." << std::endl;
    } else {
      stateFile.open(stateFileString, std::ofstream::app);
      output << "Appending to current state file." << std::endl;
    }
    return stateFile;
}
  
std::unique_ptr<Pwm_Command> makeCurrentCommand_ptr() {
  
    std::unique_ptr<Pwm_Command> currentCommand_ptr = std::make_unique<Untimed_Command>(stop_set_array);
    
    return currentCommand_ptr;
  }
  
  std::unique_ptr<Command_Interpreter_RPi5> makeCommandInterpreterPtr(std::ostream& output, std::ostream& error, std::ostream& logFile) {
    auto PhysicalPins = std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<PwmPin *> thrusterPins;
    for (auto i : PhysicalPins) {
      thrusterPins.push_back(new HardwarePwmPin(i, logFile, output, error));
    }
  
    auto wiringControl = WiringControl(output, logFile, error);
    std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr = std::make_unique<Command_Interpreter_RPi5>(
        thrusterPins, std::vector<DigitalPin *>{}, wiringControl, logFile, output, error);
    
    return commandInterpreter_ptr;
  }
  
  
int main(int argc, char *argv[]) {
    std::ostream& output = std::cout;
    std::ostream& error = std::cerr;
    std::ofstream logFilePins("PWM_LOGS.txt");
    std::ofstream stateFile = makeStateFile(output, error);

    // Set up performance and time benchmarks.
    // Set up Robot during initialization.
    rclcpp::init(argc, argv);
    SetupRobot initStateandConfig = SetupRobot();

    auto commandInterpreter_ptr = makeCommandInterpreterPtr(output, error, stateFile);
    commandInterpreter_ptr->initializePins();

    auto currentCommand_ptr = makeCurrentCommand_ptr();

    output << "Executive Main Loop Object Creation" << std::endl;
    auto mainLoopObject = std::make_shared<ExecutiveLoop>(std::move(commandInterpreter_ptr), std::move(currentCommand_ptr), stateFile, output, error);
    auto sensorsROScallback = std::make_shared<SensorsDataConfig>(mainLoopObject);

    // records false if run has not completed yet.
    bool runStatus = false;

    // these threads functions will have loops that go on for ever
    // these functions will have wait functions just in case with a queue
    // system.

    // std::jthread ReadInputsThread(&ExecutiveLoop::ReadInputs,
    // mainLoopObject); 
    // Creates a new thread for each function.

    std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);

    std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop,
                                                mainLoopObject);
    std::jthread SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
                                            mainLoopObject);

    // std::jthread
    // SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
    // mainLoopObject);
    output << "User defined threads has ran sucessfully" << std::endl;

    rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
    // auto sensorNode = std::make_shared<SensorsDataConfig>(sensorsROScallback);

    SensorsExecutor.add_node(sensorsROScallback);
    // SensorsExecutor.add_node(mainLoopObject);
    output << "ROS2 runnning" << std::endl;
    SensorsExecutor.spin();
    output << "ROS2 runnning" << std::endl;

    rclcpp::shutdown();
    output << "ROS2 exited." << std::endl;
    /*
        ReadInputsThread.join();
        UpdateStateThread.join();
        ExecutiveDecisionLoopThread.join();
        SendThrusterCommandThread.join();*/

    return 0;
}
#endif // TESTING_EXCLUDE_MAIN
