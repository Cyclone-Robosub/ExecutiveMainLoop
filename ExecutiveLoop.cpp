#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "SetupConfig/SetupRobot.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>
#include "Command_Interpreter.h"



#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std::literals;
namespace fs = std::filesystem;

#define UPDATE_WAIT_TIME 5
#define IMU_SENSOR_WAIT_TIME 2
#define NULL_SENSOR_VALUE -320000
#define FAULTY_SENSOR_VALUE -40404

// start the executive Loop
class ExecutiveLoop : public rclcpp::Node {
public:
  // Setup for all the functions should be done here.
  // Maybe we could code each function to setup on its own.
  // The functions run assuming that the inital first iteration
  // of the loop starts stage by stage with no wait.
  ExecutiveLoop() : Node("executive_main_node") {
   std::cout << "Constructor Executive Loop"<<std::endl;
    fs::path currentPath = fs::current_path();
    fs::path stateFilePath = currentPath.parent_path().parent_path();
    std::string stateFileString = std::string(stateFilePath) + "/state.csv";
    std::cout << stateFileString << std::endl;
    if (!std::filesystem::exists(stateFileString)) {
      stateFile.open(stateFileString, std::ofstream::app);

      // Append this for every new file.
      stateFile << "Time,Depth(m),Pressure, IMU Data, PWM Data" << std::endl;
      std::cout << "Created new state file." << std::endl;
    }else{
      stateFile.open(stateFileString, std::ofstream::app);
      std::cout <<"Appending to current state file"<< std::endl;
    }
    loopIsRunning = true;
    tasksCompleted = false;
    for(int i = 0; i < 8; i++){
      thrusterPins.push_back(new HardwarePwmPin(i));
      //digitalPins.push_back(new DigitalPin(5, ActiveLow));
    }
    commandInterpreter = std::make_unique<Command_Interpreter_RPi5>(thrusterPins, digitalPins);
    commandInterpreter->initializePins();


  }
  //these callback functions serve as the "read Input node in the loop"
  void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
    //  std::lock_guard<std::mutex> lock(mutex_);
    //std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
   // std::cout << "Got depth ";
   depth_pressure_msg = msg->data;
   
  }

  void imuSensorCallback(const sensor_msgs::msg::Imu &msg) {
    // std::lock_guard<std::mutex> lock(mutex_);
    //std::cout << "imu sensor\n";
    angular_velocity_x = msg.angular_velocity.x;
    angular_velocity_y = msg.angular_velocity.y;
    angular_velocity_z = msg.angular_velocity.z;
    linear_acceleration_x = msg.linear_acceleration.x;
    linear_acceleration_y = msg.linear_acceleration.y;
    linear_acceleration_z = msg.linear_acceleration.z;
    
  }
  void magCallback(const sensor_msgs::msg::MagneticField& msg)
  {
    mag_field_x = msg.magnetic_field.x;
    mag_field_y = msg.magnetic_field.y;
    mag_field_z = msg.magnetic_field.z;
  }
  void pythonCltoolCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    std::cout << "Received Int32MultiArray: ";
      int i = 0;
      int setvalue;
      std::lock_guard<std::mutex> pwm_lock(pwm_mutex);
      for (int32_t value : msg->data) {
        setvalue = (int)value;
        our_pwm_array.pwm_signals[i] = setvalue;
        std::cout << setvalue;
        i++;
      }
      i = 0;
      std::cout << std::endl;
      
  }

/*
  void readInputs() {
    while (loopIsRunning) {
      //std::lock_guard<std::mutex> lock(mutex_);
      // Have to check what the msg is saying.
      // Parse msg data. Put the research data into a vector or var iables.
      // IMU data will probably go into vector.
      // IF needed we can use parameters with ROS if a lot of different types
      // of data. one part of message has to the be imu and the other part has
      // to be the depth. Assuming I have it right. Need to read
      // multithreading with mutex condition and lock pushing to variables. or
      // anyway of setting it

    }
  }
*/
  // get a notification here
  void updateState() {
    std::cout << "UpdateState" << std::endl;
    while (loopIsRunning) {
      // Get the variables and put it into the state file.
      // timestamped every 0.1 seconds.
      std::lock_guard<std::mutex> sensorDataLock(sensor_mutex);
      std::unique_lock<std::mutex> pwmValuesLock(pwm_mutex);
        //try ownslock for future testing
      stateFile << getCurrentDateTime() << ",";
      
    //    std::cout << depth_msg << " updateStateLocation" << " \n";
        stateFile << depth_pressure_msg << ",";
        stateFile << angular_velocity_x << "," << angular_velocity_y  << "," << angular_velocity_z << "," << linear_acceleration_x << "," << linear_acceleration_y << "," << linear_acceleration_z << ","; 
        stateFile << mag_field_x << "," << mag_field_y << "," << mag_field_z << "," ;
      
      stateFile << ",[";
        for(auto i : our_pwm_array.pwm_signals){
          stateFile << i << ",";
        }
        stateFile << "],";
        stateFile << "\n";
      if(stateFile.tellp() > 800){
        stateFile.flush();
        stateFile.clear();
        stateFile.seekp(0);

      }
      std::this_thread::sleep_for(
            std::chrono::milliseconds(UPDATE_WAIT_TIME));
                  //TODO: Need to see William's code to put PWM here in the status file.
    }
  }

  void executeDecisionLoop() {
    std::string executeType;
    while (loopIsRunning) {
      if(userinput == "end"){
        executeFailCommands();
        std::cout << "User Interrupted Executive Loop" << std::endl;
        break;
      }
      executeType = "blind_execute";
      sendThrusterCommands(executeType);
      std::cin >> userinput;

      //Need to see William's python code to move foward.

    }
    // if all decisions/tasks are done, make tasksCompleted true;
  }

  // Sends Commands to Thruster Queue
  void sendThrusterCommands(std::string typeOfExecute) {
      if(typeOfExecute == "blind_execute"){
        std::ofstream logFilePins;
        CommandComponent commandComponents;
       // our_pwm_array.pwm_signals = inputPWM;
        commandComponents.thruster_pwms = our_pwm_array;
        //setup ROS topic for duration
        commandComponents.duration = std::chrono::milliseconds(50);
        commandInterpreter->blind_execute(commandComponents, logFilePins);
      }
      // send it back to William's code.
  }

  bool returnStatus() { return loopIsRunning; }
  bool returntasksCompleted() { return tasksCompleted; }
  void executeFailCommands() {
    std::lock_guard<std::mutex> stateFileLock(sensor_mutex);
    stateFile << std::endl;
    stateFile.close();
    //loopIsRunning = false;
    std::cout << "Shutting down Executive Loop, sensors are still reading." <<std::endl;
  }
  /*
  void ShutdownRobot(){
    if(FuncFailCommExecuted){
      //shutdown opreations
    }
  }*/

private:
rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      python_cltool_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  float angular_velocity_x = NULL_SENSOR_VALUE;
     float angular_velocity_y = NULL_SENSOR_VALUE;
     float angular_velocity_z = NULL_SENSOR_VALUE;
     float linear_acceleration_x = NULL_SENSOR_VALUE;
     float linear_acceleration_y = NULL_SENSOR_VALUE;
     float linear_acceleration_z = NULL_SENSOR_VALUE;

     float mag_field_x = NULL_SENSOR_VALUE;
      float mag_field_y = NULL_SENSOR_VALUE; 
      float mag_field_z = NULL_SENSOR_VALUE;

  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter;
  std::vector<PwmPin*> thrusterPins;
  std::vector<DigitalPin*> digitalPins;
  int inputPWM[8] = {15, 15, 15, 15, 15, 15, 15, 15};
  pwm_array our_pwm_array;
  std::ofstream stateFile;
  std::mutex sensor_mutex;
  std::mutex pwm_mutex;
  std::string depth_pressure_msg ="Depth Sensor Not Started Yet";
  std::string imu_msg;
   std::vector<float> imu_data;
  float depth;
  bool loopIsRunning = true;
  bool tasksCompleted;
  std::string userinput;

  std::string getCurrentDateTime() {
    time_t now = time(0);
    tm *localTime = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
    return std::string(buffer);
  }
};

class SensorsData : public rclcpp::Node {
public:
  SensorsData(std::shared_ptr<ExecutiveLoop> mainLoopObject)
      : Node("sensorsNode") {
    callbackDepthPressure = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackIMU = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto commandOptions = rclcpp::SubscriptionOptions();
    commandOptions.callback_group = callbackDepthPressure;
    auto depthPressureOptions = rclcpp::SubscriptionOptions();
    depthPressureOptions.callback_group = callbackDepthPressure;
    auto imuOptions = rclcpp::SubscriptionOptions();
    imuOptions.callback_group = callbackIMU;
    std::cout << "Creating sensors subscriptions\n";
    auto magOptions = rclcpp::SubscriptionOptions();
    magOptions.callback_group = callbackIMU;

    depth_pressure_sensor_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthPressureSensorData", rclcpp::QoS(5),
            std::bind(&ExecutiveLoop::depthPressureSensorCallback, mainLoopObject,
                      std::placeholders::_1),
            depthPressureOptions);

    // Priority
    // Need to input IMU initialization with ROS.

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::QoS(5),
        std::bind(&ExecutiveLoop::imuSensorCallback, mainLoopObject,
                  std::placeholders::_1),
        imuOptions);
    mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "magtopic", rclcpp::QoS(5),
        std::bind(&ExecutiveLoop::magCallback, mainLoopObject,
                  std::placeholders::_1),
        magOptions);
    pythonCltool_subscription =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "python_Manual_cltool_topic", 10,
            std::bind(&ExecutiveLoop::pythonCltoolCallback, mainLoopObject,
                      std::placeholders::_1),
                    commandOptions);
  }


private:
  rclcpp::CallbackGroup::SharedPtr callbackDepthPressure;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      depth_pressure_sensor_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pythonCltool_subscription;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]) {
  // setup performance and time benchmarks.
  //  setup Robot during initialization.
  rclcpp::init(argc, argv);
  SetupRobot initStateandConfig = SetupRobot();
  // ExecutiveLoop Think about object by reference or value passing
  std::cout << "Executive Main Loop Object Creation" <<std::endl;
  std::shared_ptr<ExecutiveLoop> mainLoopObject = std::make_shared<ExecutiveLoop>();
  std::shared_ptr<SensorsData> sensorsROScallback = std::make_shared<SensorsData>(mainLoopObject);
  // ExecutiveLoop mainLoopObject = ExecutiveLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;
  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue
  // system.

  // std::jthread ReadInputsThread(&ExecutiveLoop::ReadInputs,
  // mainLoopObject); Creates a new thread for each node. Need to check if it does


  std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);

  std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop, mainLoopObject);
    // Note: We can join these two threads above and bottom if Raspberry PI
    // really does not like multithreading.
    //This is now the case ^.
 // std::jthread SendThrusterCommandsThread(&ExecutiveLoop::sendThrusterCommands, mainLoopObject);
  std::cout << "User defined threads has ran sucessfully" << std::endl;

  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
 // auto sensorNode = std::make_shared<SensorsData>(sensorsROScallback);

  SensorsExecutor.add_node(sensorsROScallback);
  //SensorsExecutor.add_node(mainLoopObject);
  std::cout << "ROS2 runnning" << std::endl;
  SensorsExecutor.spin();
  std::cout << "ROS2 runnning" << std::endl;

  rclcpp::shutdown();
  std::cout << "ROS2 exited." << std::endl;
  /*
    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandsThread.join();*/

  return 0;
}
#endif // TESTING_EXCLUDE_MAIN