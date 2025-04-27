#include <atomic>
#include <chrono>
#include <condition_variable>
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

#define UPDATE_WAIT_TIME 5
#define IMU_SENSOR_WAIT_TIME 2
#define NULL_SENSOR_VALUE -320000
#define FAULTY_SENSOR_VALUE -40404

// start the executive Loop
class ExecutiveLoop : public rclcpp::Node {
public:
  // Setup for all the functions should be done here.
  // Setup everything so that when the threads startup, it can run its tasks.
  ExecutiveLoop() : Node("executive_main_node") {
    std::cout << "Constructor Executive Loop" << std::endl;

    //Setting a stop set in the beginnning of startup -> needs to be better.
    pwm_array zero_set_array;
    for (int i = 0; i < 8; i++) {
      zero_set_array.pwm_signals[i] = 1500;
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
    auto PhysicalPins = std::vector<int>{2, 3, 4, 5, 6, 7, 8, 9};
    for (auto i : PhysicalPins) {
      thrusterPins.push_back(new HardwarePwmPin(i));
      // digitalPins.push_back(new DigitalPin(5, ActiveLow));
    }

    //Setup the Command Interpreter's pins with the physical pins.
    commandInterpreter_ptr = std::make_unique<Command_Interpreter_RPi5>(
        thrusterPins, std::vector<DigitalPin *>{});
    commandInterpreter_ptr->initializePins();
   // commandInterpreter_ptr->readPins();
  }
  // these callback functions serve as the "read Input node in the loop"
  //Feel free to later push these into the Sensors Class, but make sure ExecutiveLoop can still access through memory its needed fields.

  void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
    isManualEnabled = msg->data;
    if (isManualEnabled) {
      std::cout << "Manual Control Enabled" << std::endl;
      // need to add to here later.
    } else {
      std::cout << "Manual Control Disabled" << std::endl;
      std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
      std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
      std::swap(ManualPWMQueue, empty);
      std::cout << "Manual Command Current Override -> Deleted Queue"
                << std::endl;
      sizeQueue = 0;
    }
    Change_Manual.notify_all();
  }
  //This should only clear the queue
  void ManualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::unique_lock<std::mutex> Manual_Lock(Manual_Override_mutex);
    isManualOverride = msg->data;
    if (isManualOverride) {
      std::lock_guard<std::mutex> QueueLock(Queue_pwm_mutex);
      std::queue<std::pair<pwm_array, std::chrono::milliseconds>> empty;
      std::swap(ManualPWMQueue, empty);
      std::cout << "Manual Command Current Override -> Deleted Queue"
                << std::endl;
      sizeQueue = 0;
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

  //First get the PWM Array. Then Allow the duration callback to execute and pair the array with the duration. Then push it onto the queue for ExecuteDecision. Notify every time we allow either Duration or Execute to use the queue for chain of execution. 
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
    SendToDuration_change.notify_all();
    pwm_lock.unlock();
  }
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    std::unique_lock<std::mutex> duration_lock(array_duration_sync_mutex,
                                               std::defer_lock);
    SendToDuration_change.wait(duration_lock,
                               [this] { return AllowDurationSync; });
    std::cout << "Getting duration" << std::endl;
    auto duration_int_pwm = msg->data;
    std::chrono::milliseconds durationMS;
    bool isgivenTimed = false;
    // duration_int_pwm = std::stoi(duration_pwm);
    switch (duration_int_pwm) {
    case -1:
    //PWM
      durationMS = std::chrono::milliseconds(9999999999);
      std::cout << durationMS << std::endl;
      break;
    default:
    //TIMED PWM
      durationMS = std::chrono::milliseconds(duration_int_pwm * 1000);
      isgivenTimed = true;
      std::cout << durationMS << std::endl;
      break;
    }
    std::unique_lock<std::mutex> Queue_sync_lock(Queue_pwm_mutex);
    ManualPWMQueue.push(std::make_pair(given_array, durationMS));
    sizeQueue++;
    if(isgivenTimed){
      pwm_array stop_set_array;
      for (int i = 0; i < 8; i++) {
        stop_set_array.pwm_signals[i] = 1500;
      }
      std::pair<pwm_array, std::chrono::milliseconds> stop_set_pair(
          stop_set_array, std::chrono::milliseconds(99999999));
      ManualPWMQueue.push(stop_set_pair);
      sizeQueue++;
    }
    AllowDurationSync = false;
    PWM_cond_change.notify_all();
    std::cout << "Pushed to queue, Duration: " << duration_int_pwm << std::endl;
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
  void updateState() {
    std::cout << "UpdateState" << std::endl;
    while (loopIsRunning) {
      // Get the variables and put it into the state file.
      // timestamped every 0.1 seconds.
      std::lock_guard<std::mutex> sensorDataLock(sensor_mutex);
      // try ownslock for future testing
      stateFile << getCurrentDateTime() << ",";

      //    std::cout << depth_msg << " updateStateLocation" << " \n";
      stateFile << depth_pressure_msg;
      std::unique_lock<std::mutex> IMUlock(imu_mutex);
      stateFile << " ";
      stateFile << angular_velocity_x << ", " << angular_velocity_y << ", "
                << angular_velocity_z << ", " << linear_acceleration_x << ", "
                << linear_acceleration_y << ", " << linear_acceleration_z << ", ";
      IMUlock.unlock();
      stateFile << mag_field_x << ", " << mag_field_y << ", " << mag_field_z
                << ", PWM :[";

      std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
      for (auto i : currentPWMandDuration_ptr->first.pwm_signals) {
        stateFile << i << ", ";
      }
      stateFile << "],";
      pwmValuesLock.unlock();
      stateFile << "\n";
      if (stateFile.tellp() > 200) {
        stateFile.flush();
        stateFile.clear();
        stateFile.seekp(0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
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
      // test these two lines of code
      std::unique_lock<std::mutex> Manual_Lock(Manual_Mutex);
      // Change_Manual.wait(Manual_Lock, [this] { return isManualEnabled; });

      //CHECK CL-TOOL IS CONTROLLING
      if (isManualEnabled) {
      //  Manual_Lock.unlock();
     //   std::cout << "Manual Enabled" << std::endl;
        //if typed in CL-Tool was tcs.override()
      std::unique_lock<std::mutex> Manual_Override_Lock(Manual_Override_mutex);
        if (isManualOverride) {
          //if the sendThrusterCommand is currently running or has a task.
          if (isRunningThrusterCommand) {
            std::cout << "manual override" << std::endl;
            override();
            //override was successful; revert back.
            isManualOverride = false;
          }
        }
        typeOfExecute = "blind_execute";
        //std::cout << "Getting the lock for queue." << std::endl;
        std::unique_lock<std::mutex> QueuepwmValuesLock(Queue_pwm_mutex,
                                                        std::defer_lock);
       // std::cout << "Got the queue lock, looking at the queue." << std::endl;
        PWM_cond_change.wait(QueuepwmValuesLock,
                             [this] { return !(sizeQueue == 0); });
        //std::cout << "Something is in the queue." << std::endl;
        //Make sure isCurrnetCommandTimedPWM is only changed by Executive DecisionLoop or else put a mutex on it.
        if (!isCurrentCommandTimedPWM) {
          override();
          std::cout << "Executive Decision: Replace Current PWM Command" << std::endl;
          if (ManualPWMQueue.front().second >=
              std::chrono::milliseconds(99999999)) {
            isCurrentCommandTimedPWM = false;
          } else {
            isCurrentCommandTimedPWM = true;
          }
          std::unique_lock<std::mutex> CurrentpwmValuesLock(
              current_PWM_duration_mutex);
          currentPWMandDuration_ptr =
              std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
                  ManualPWMQueue.front());
          CurrentpwmValuesLock.unlock();
         // std::cout << " Executor Decision: Replace PWM" << std::endl;
          std::unique_lock<std::mutex> thrusterCommandLock(thruster_mutex);
          isRunningThrusterCommand = true;
          thrusterCommandLock.unlock();
          ManualPWMQueue.pop();
          sizeQueue--;
        }
        // Add comment here below and above.
        else if (!isRunningThrusterCommand) {
          std::cout << "2 Executor Decision: Needs a Command" << std::endl;
          if (ManualPWMQueue.front().second >=
              std::chrono::milliseconds(9999999)) {
            isCurrentCommandTimedPWM = false;
          } else {
            isCurrentCommandTimedPWM = true;
          }
          std::unique_lock<std::mutex> CurrentpwmValuesLock(
              current_PWM_duration_mutex);
          currentPWMandDuration_ptr =
              std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
                  ManualPWMQueue.front());
          std::unique_lock<std::mutex> statusThruster(thruster_mutex);
          isRunningThrusterCommand = true;
          statusThruster.unlock();
          CurrentpwmValuesLock.unlock();
          std::cout << "3 executor decision: Gave New Command" << std::endl;
          //std::unique_lock<std::mutex> thrusterCommandLock(thruster_mutex);
          //thrusterCommandLock.unlock();
          ManualPWMQueue.pop();
          sizeQueue--;
        }else{

        }
      }
    }
    // Need to see William's python code to move foward.
  }
  // if all decisions/tasks are done, make tasksCompleted true;

  // Sends Commands to Thrusters with CommandInterpreter
  void sendThrusterCommand() {
    std::ofstream logFilePins("PWM_LOGS.txt");
    while (loopIsRunning) {
      if (typeOfExecute == "blind_execute") {
        CommandComponent commandComponent;
        // our_pwm_array.pwm_signals = inputPWM;
        if (isRunningThrusterCommand) {
          std::cout << "Send Thruster Command is doing its job" << std::endl;
          std::unique_lock<std::mutex> CurrentpwmValuesLock(
              current_PWM_duration_mutex);
          commandComponent.thruster_pwms = currentPWMandDuration_ptr->first;
          // setup ROS topic for duration
          commandComponent.duration = currentPWMandDuration_ptr->second;
          CurrentpwmValuesLock.unlock();
          commandInterpreter_ptr->blind_execute(commandComponent, logFilePins);
          std::cout << "Finished Thruster Command\n" << std::endl;

          std::unique_lock<std::mutex> statusThruster(thruster_mutex);
          isRunningThrusterCommand = false;
          statusThruster.unlock();
          // Thruster_cond_change.notify_all();
          // completed
        }
      }
    }
    
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
  //NOTE: there are going to be a lot of unused variables, please remove in the future.
  bool isManualEnabled = false;
  bool isManualOverride = false;
  bool isRunningThrusterCommand = false;
  bool isCurrentCommandTimedPWM = false;
  bool AllowDurationSync = false;
  std::mutex thruster_mutex;
  std::mutex array_duration_sync_mutex;
  std::mutex Manual_Mutex;
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

  //The current PWM and duration ptr will and should always have a value regardless of what Executive DecisionLoop or Send Thrusters want. However, SendThrusters can "finish" a current PWM and duration and will say that it wants a new command, but it can be the same current PWM if ExecutiveDecision decides so. Executive Decision (on its own thread) will see that SendThrusters is not running a command and give it a new current PWM. This is made so that Executive Decision has the chance to give PWM a new Command if the current one is a timedPWM. In Later uses, the State file should use the current PWM that the Send Thruster is using.
  std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>>
      currentPWMandDuration_ptr;
  // bool isQueuePWMEmpty = true;
  std::ofstream stateFile;
  std::mutex sensor_mutex;
  std::mutex Queue_pwm_mutex;
  std::mutex imu_mutex;
  std::mutex ThrusterCommand_mutex;
  std::mutex Manual_Override_mutex;
  std::mutex current_PWM_duration_mutex;
  std::condition_variable SendToDuration_change;
  std::condition_variable PWM_cond_change;
  std::condition_variable Thruster_cond_change;
  std::condition_variable Change_Manual;
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
  //If a task is currently running, stop this task and set a zero pair to clear everything. Execute Decision Loop should then be able to decide what to do next.
  void override() {
    if (isRunningThrusterCommand) {
      //May have to have a mutex for this ptr.
      commandInterpreter_ptr->interruptBlind_Execute();
      pwm_array zero_set_array;
      for (int i = 0; i < 8; i++) {
        zero_set_array.pwm_signals[i] = 0;
      }
      std::pair<pwm_array, std::chrono::milliseconds> zero_set_pair(
          zero_set_array, std::chrono::milliseconds(100));
      std::unique_lock<std::mutex> pwmValuesLock(current_PWM_duration_mutex);
      currentPWMandDuration_ptr =
          std::make_shared<std::pair<pwm_array, std::chrono::milliseconds>>(
              zero_set_pair);
      pwmValuesLock.unlock();
      isCurrentCommandTimedPWM = false;
      std::lock_guard<std::mutex> statusThrusterLock(thruster_mutex);
      isRunningThrusterCommand = false;
    }
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
    callbackManual =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
    ManualToggleOptions.callback_group = callbackManual;
    auto ManualOverride = rclcpp::SubscriptionOptions();
    ManualOverride.callback_group = callbackManual;

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

              //Please implement Kory's data as soon as possible
    /*did_ins_subscription =
    this->create_subscription<sensor_msgs::msg::MagneticField>(
        "mag", rclcpp::QoS(5),
        std::bind(&ExecutiveLoop::, mainLoopObject,
                  std::placeholders::_1),
        did_ins_Options);*/
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