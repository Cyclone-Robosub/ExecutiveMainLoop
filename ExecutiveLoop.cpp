#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "Command_Interpreter.h"
#include "SetupConfig/SetupRobot.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std::literals;
namespace fs = std::filesystem;

#define UPDATE_WAIT_TIME 3
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
    std::cout << "Constructor Executive Loop" << std::endl;
    pwm_array zero_set_array;
    for (int i = 0; i < 8; i++) {
      zero_set_array.pwm_signals[i] = 0;
    }
    std::pair<pwm_array, std::chrono::milliseconds> zero_set_pair(
        zero_set_array, std::chrono::milliseconds(99999999));
    std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
    currentPWMandDuration_ptr =
        std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
            zero_set_pair);
            pwmValuesLock.unlock();
    // State file creation or appending
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

    // Status of Loop
    loopIsRunning = true;
    tasksCompleted = false;

    // Setup Pins
    auto PhysicalPins = std::vector<int>{4, 5, 2, 3, 9, 7, 8, 6};
    for (auto i : PhysicalPins) {
      thrusterPins.push_back(new HardwarePwmPin(i));
      // digitalPins.push_back(new DigitalPin(5, ActiveLow));
    }
    commandInterpreter_ptr =
        std::make_unique<Command_Interpreter_RPi5>(thrusterPins, digitalPins);
    commandInterpreter_ptr->initializePins();
  }
  // these callback functions serve as the "read Input node in the loop

  void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    isManualEnabled = msg->data;
    if (isManualEnabled) {
      std::cout << "Manual Control Enabled" << std::endl;
    } else {
      std::cout << "Manual Control Disabled" << std::endl;
    }
  }
  void ManualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    isManualOverride = msg->data;
    if (isManualOverride) {
      std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
      std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
      std::swap(ManualPWMQueue, empty);
      std::cout << "Manual Command Current Override -> Deleted Queue"
                << std::endl;
    }
  }
  void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
    //  std::lock_guard<std::mutex> lock(mutex_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
    // std::cout << "Got depth ";
    depth_pressure_msg = msg->data;
  }

  void imuSensorCallback(const sensor_msgs::msg::Imu &msg) {

    std::lock_guard<std::mutex> CallBacklock(imu_mutex);
    // std::cout << "imu sensor\n";
    angular_velocity_x = msg.angular_velocity.x;
    angular_velocity_y = msg.angular_velocity.y;
    angular_velocity_z = msg.angular_velocity.z;
    linear_acceleration_x = msg.linear_acceleration.x;
    linear_acceleration_y = msg.linear_acceleration.y;
    linear_acceleration_z = msg.linear_acceleration.z;
  }
  void magCallback(const sensor_msgs::msg::MagneticField &msg) {
    mag_field_x = msg.magnetic_field.x;
    mag_field_y = msg.magnetic_field.y;
    mag_field_z = msg.magnetic_field.z;
  }
  void PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    std::cout << "Received Int32MultiArray: ";
    int i = 0;
    int setvalue;
    std::unique_lock<std::mutex> pwm_lock(array_duration_sync_mutex);
    // pwm_array receivedArray;
    for (int32_t value : msg->data) {
      setvalue = (int)value;
      given_array.pwm_signals[i] = setvalue;
      std::cout << given_array.pwm_signals[i];
      ++i;
    }
    AllowDurationSync = true;
    std::cout << std::endl;
    pwm_lock.unlock();
    PWM_cond_change.notify_all();
  }
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    std::unique_lock<std::mutex> duration_lock(array_duration_sync_mutex,
                                               std::defer_lock);
    PWM_cond_change.wait(duration_lock, [this] { return AllowDurationSync; });
    std::cout << "Getting duration" << std::endl;
    auto duration_int_pwm = msg->data;
    std::chrono::milliseconds durationMS;
    // duration_int_pwm = std::stoi(duration_pwm);
    switch (duration_int_pwm) {
    case -1:
      durationMS = std::chrono::milliseconds(9999999999);
      break;
    default:
      durationMS = std::chrono::milliseconds(duration_int_pwm * 1000);
      break;
    }
    std::unique_lock<std::mutex> Queue_sync_lock(Queue_pwm_mutex);
    ManualPWMQueue.push(std::make_pair(given_array, durationMS));
    sizeQueue++;
    Queue_sync_lock.unlock();
    std::cout << "Pushed to queue, Duration: " << duration_int_pwm << std::endl;
    AllowDurationSync = false;
    duration_lock.unlock();
  }
  /*
    void readInputs() {
      while (loopIsRunning) {
        //std::lock_guard<std::mutex> lock(mutex_);
        // Have to check what the msg is saying.
        // Parse msg data. Put the research data into a vector or var
    iables.
        // IMU data will probably go into vector.
        // IF needed we can use parameters with ROS if a lot of different
    types
        // of data. one part of message has to the be imu and the other part
    has
        // to be the depth. Assuming I have it right. Need to read
        // multithreading with mutex condition and lock pushing to
    variables. or
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
      // try ownslock for future testing
      stateFile << getCurrentDateTime() << ",";

      //    std::cout << depth_msg << " updateStateLocation" << " \n";

      std::unique_lock<std::mutex> IMUlock(imu_mutex);
      stateFile << depth_pressure_msg << ", IMU:";
      stateFile << angular_velocity_x << "," << angular_velocity_y << ","
                << angular_velocity_z << "," << linear_acceleration_x << ","
                << linear_acceleration_y << "," << linear_acceleration_z << ",";
      IMUlock.unlock();
      stateFile << mag_field_x << "," << mag_field_y << "," << mag_field_z
                << ", PWM :[";

      std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex,
                                                 std::defer_lock);
      if (pwmValuesLock.try_lock()) {
        for (auto i : currentPWMandDuration_ptr->first.pwm_signals) {
          stateFile << i << ",";
        }
        stateFile << "],";
        // lock automatically releases when pwmValuesLock goes out of scope
      } else {
        std::cerr << "Could not acquire lock on current_PWM_duration_mutex\n";
      }
      stateFile << "\n";
      if (stateFile.tellp() > 200) {
        stateFile.flush();
        stateFile.clear();
        stateFile.seekp(0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
      // TODO: Need to see William's code to put PWM here in the status file.
    }
  }

  void executeDecisionLoop() {
    while (loopIsRunning) {
      // Control Loop from Simulink
      /*if (userinput == "end") {
      executeFailCommands();
      std::cout << "User Interrupted Executive Loop" << std::endl;
      break;
    }*/
      if(isManualEnabled){
        
      if (isManualOverride) {
        override();
      }
      typeOfExecute = "blind_execute";
      std::unique_lock<std::mutex> pwmValuesLock(Queue_pwm_mutex,
                                                 std::defer_lock);
      PWM_cond_change.wait(pwmValuesLock, [this] { return !(sizeQueue == 0); });
      std::unique_lock<std::mutex> thrusterCommandLock(thruster_mutex);
      std::cout << "Here EXECUTDECISION LOOP STARTING AFTER WAIT" << std::endl;
      if (isRunningThrusterCommand) {
        if (!isCurrentCommandTimedPWM) {
          override();
          if (ManualPWMQueue.front().second >=
              std::chrono::milliseconds(99999999)) {
            isCurrentCommandTimedPWM = false;
          } else {
            isCurrentCommandTimedPWM = true;
          }
          std::unique_lock<std::mutex> pwmValuesLock(
              current_PWM_duration_mutex);
          currentPWMandDuration_ptr =
              std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
                  ManualPWMQueue.front());
                pwmValuesLock.unlock();
          isRunningThrusterCommand = true;
          ManualPWMQueue.pop();
          thrusterCommandLock.unlock();
          pwmValuesLock.unlock();
        }
        // Add comment here below and above.
      } else {
        if (ManualPWMQueue.front().second >=
            std::chrono::milliseconds(99999999)) {
          isCurrentCommandTimedPWM = false;
        } else {
          isCurrentCommandTimedPWM = true;
        }
        std::unique_lock<std::mutex> pwmarrayValuesLock(current_PWM_duration_mutex);
        currentPWMandDuration_ptr =
            std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
                ManualPWMQueue.front());
                pwmarrayValuesLock.unlock();
        isRunningThrusterCommand = true;
        ManualPWMQueue.pop();
        thrusterCommandLock.unlock();
        pwmValuesLock.unlock();
      }
    }
    }
    // Need to see William's python code to move foward.
  }
  // if all decisions/tasks are done, make tasksCompleted true;

  // Sends Commands to Thruster Queue
  void sendThrusterCommand() {
    while (loopIsRunning) {
      if (typeOfExecute == "blind_execute") {
        std::ofstream logFilePins;
        CommandComponent commandComponent;
        // our_pwm_array.pwm_signals = inputPWM;
        if (isRunningThrusterCommand) {
          std::unique_lock<std::mutex> statusThruster(thruster_mutex);
          commandComponent.thruster_pwms = currentPWMandDuration_ptr->first;
          // setup ROS topic for duration
          commandComponent.duration = currentPWMandDuration_ptr->second;
          commandInterpreter_ptr->blind_execute(commandComponent, logFilePins);
          // Thruster_cond_change.notify_all();
          // completed
          isRunningThrusterCommand = false;
          statusThruster.unlock();
        }
      }
    }
    // send it back to William's code.
  }

  bool returnStatus() { return loopIsRunning; }
  bool returntasksCompleted() { return tasksCompleted; }
  void executeFailCommands() {
    std::lock_guard<std::mutex> stateFileLock(sensor_mutex);
    stateFile << std::endl;
    stateFile.close();
    // loopIsRunning = false;
    std::cout << "Shutting down Executive Loop, sensors are still reading."
              << std::endl;
  }
  /*
  void ShutdownRobot(){
    if(FuncFailCommExecuted){
      //shutdown opreations
    }
  }*/

private:
  bool isManualEnabled = false;
  bool isManualOverride = false;
  bool isRunningThrusterCommand = false;
  bool isCurrentCommandTimedPWM = false;
  bool AllowDurationSync = false;
  std::mutex thruster_mutex;
  std::mutex array_duration_sync_mutex;
  unsigned int sizeQueue = 0;

  float angular_velocity_x = NULL_SENSOR_VALUE;
  float angular_velocity_y = NULL_SENSOR_VALUE;
  float angular_velocity_z = NULL_SENSOR_VALUE;
  float linear_acceleration_x = NULL_SENSOR_VALUE;
  float linear_acceleration_y = NULL_SENSOR_VALUE;
  float linear_acceleration_z = NULL_SENSOR_VALUE;

  float mag_field_x = NULL_SENSOR_VALUE;
  float mag_field_y = NULL_SENSOR_VALUE;
  float mag_field_z = NULL_SENSOR_VALUE;

  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr;
  std::vector<PwmPin *> thrusterPins;
  std::vector<DigitalPin *> digitalPins;
  pwm_array our_pwm_array;
  std::queue<std::pair<pwm_array, std::chrono::milliseconds>> ManualPWMQueue;

  pwm_array given_array;
  std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>>
      currentPWMandDuration_ptr;
  // bool isQueuePWMEmpty = true;
  std::ofstream stateFile;
  std::mutex sensor_mutex;
  std::mutex Queue_pwm_mutex;
  std::mutex imu_mutex;
  std::mutex ThrusterCommand_mutex;
  std::mutex current_PWM_duration_mutex;
  std::condition_variable PWM_cond_change;
  std::condition_variable Thruster_cond_change;
  std::string depth_pressure_msg = "Depth Sensor Not Started Yet";
  std::string imu_msg;
  std::vector<float> imu_data;
  float depth = NULL_SENSOR_VALUE;
  float pressure = NULL_SENSOR_VALUE;
  bool loopIsRunning;
  bool tasksCompleted;
  std::string userinput;
  int duration_int_pwm;
  std::string typeOfExecute;

  std::string getCurrentDateTime() {
    time_t now = time(0);
    tm *localTime = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
    return std::string(buffer);
  }
  void override() {
    commandInterpreter_ptr->interruptBlind_Execute();
    pwm_array zero_set_array;
    for (int i = 0; i < 8; i++) {
      zero_set_array.pwm_signals[i] = 0;
    }
    std::pair<pwm_array, std::chrono::milliseconds> zero_set_pair(
        zero_set_array, std::chrono::milliseconds(99999999));
    std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
    currentPWMandDuration_ptr =
        std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
            zero_set_pair);
            pwmValuesLock.unlock();
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
    callbackClTool =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callbackManual = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto commandOptions = rclcpp::SubscriptionOptions();
    commandOptions.callback_group = callbackClTool;
    auto durationOptions = rclcpp::SubscriptionOptions();
    durationOptions.callback_group = callbackClTool;
    auto depthPressureOptions = rclcpp::SubscriptionOptions();
    depthPressureOptions.callback_group = callbackDepthPressure;
    auto imuOptions = rclcpp::SubscriptionOptions();
    imuOptions.callback_group = callbackIMU;
    std::cout << "Creating sensors subscriptions\n";
    auto magOptions = rclcpp::SubscriptionOptions();
    magOptions.callback_group = callbackIMU;
    auto ManualToggleOptions = rclcpp::SubscriptionOptions();
    ManualToggleOptions.callback_group = callbackClTool;
    auto ManualOverride = rclcpp::SubscriptionOptions();
    ManualOverride.callback_group = callbackClTool;

    depth_pressure_sensor_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthPressureSensorData", rclcpp::QoS(5),
            std::bind(&ExecutiveLoop::depthPressureSensorCallback,
                      mainLoopObject, std::placeholders::_1),
            depthPressureOptions);

    // Priority
    // Need to input IMU initialization with ROS.

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::QoS(5),
        std::bind(&ExecutiveLoop::imuSensorCallback, mainLoopObject,
                  std::placeholders::_1),
        imuOptions);
    mag_subscription_ =
        this->create_subscription<sensor_msgs::msg::MagneticField>(
            "mag", rclcpp::QoS(5),
            std::bind(&ExecutiveLoop::magCallback, mainLoopObject,
                      std::placeholders::_1),
            magOptions);
    /*
did_ins_subscription =
this->create_subscription<sensor_msgs::msg::MagneticField>(
    "mag", rclcpp::QoS(5),
    std::bind(&ExecutiveLoop::magCallback, mainLoopObject,
              std::placeholders::_1),
    magOptions);*/
    CLTool_subscription_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "array_Cltool_topic", rclcpp::QoS(10),
            std::bind(&ExecutiveLoop::PWMArrayCallback, mainLoopObject,
                      std::placeholders::_1),
            commandOptions);
    duration_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "duration_Cltool_topic", rclcpp::QoS(10),
        std::bind(&ExecutiveLoop::durationCallback, mainLoopObject,
                  std::placeholders::_1),
        durationOptions);
    Manual_Control_sub = this->create_subscription<std_msgs::msg::Bool>(
        "manual_toggle_switch", rclcpp::QoS(10),
        std::bind(&ExecutiveLoop::ManualControlCallback, mainLoopObject,
                  std::placeholders::_1),
        ManualToggleOptions);
    Manual_Override_sub = this->create_subscription<std_msgs::msg::Bool>(
        "manualOverride", rclcpp::QoS(4),
        std::bind(&ExecutiveLoop::ManualOverrideCallback, mainLoopObject,
                  std::placeholders::_1),
        ManualOverride);
  }

private:
  rclcpp::CallbackGroup::SharedPtr callbackDepthPressure;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::CallbackGroup::SharedPtr callbackClTool;
  rclcpp::CallbackGroup::SharedPtr callbackManual;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      depth_pressure_sensor_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
      mag_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      CLTool_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Control_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Override_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr duration_subscription_;
};

#ifndef TESTING_EXCLUDE_MAIN
int main(int argc, char *argv[]) {
  // setup performance and time benchmarks.
  //  setup Robot during initialization.
  rclcpp::init(argc, argv);
  SetupRobot initStateandConfig = SetupRobot();

  // ExecutiveLoop Think about object by reference or value passing
  std::cout << "Executive Main Loop Object Creation" << std::endl;
  std::shared_ptr<ExecutiveLoop> mainLoopObject =
      std::make_shared<ExecutiveLoop>();
  std::shared_ptr<SensorsData> sensorsROScallback =
      std::make_shared<SensorsData>(mainLoopObject);
  // ExecutiveLoop mainLoopObject = ExecutiveLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;
  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue
  // system.

  // std::jthread ReadInputsThread(&ExecutiveLoop::ReadInputs,
  // mainLoopObject); Creates a new thread for each node. Need to check if it
  // does

  std::jthread UpdateStateThread(&ExecutiveLoop::updateState, mainLoopObject);

  std::jthread ExecutiveDecisionLoopThread(&ExecutiveLoop::executeDecisionLoop,
                                           mainLoopObject);
  std::jthread SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
                                         mainLoopObject);
  // Note: We can join these two threads above and bottom if Raspberry PI
  // really does not like multithreading.
  // This is now the case ^.
  // std::jthread
  // SendThrusterCommandThread(&ExecutiveLoop::sendThrusterCommand,
  // mainLoopObject);
  std::cout << "User defined threads has ran sucessfully" << std::endl;

  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
  // auto sensorNode = std::make_shared<SensorsData>(sensorsROScallback);

  SensorsExecutor.add_node(sensorsROScallback);
  // SensorsExecutor.add_node(mainLoopObject);
  std::cout << "ROS2 runnning" << std::endl;
  SensorsExecutor.spin();
  std::cout << "ROS2 runnning" << std::endl;

  rclcpp::shutdown();
  std::cout << "ROS2 exited." << std::endl;
  /*
    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandThread.join();*/

  return 0;
}
#endif // TESTING_EXCLUDE_MAIN