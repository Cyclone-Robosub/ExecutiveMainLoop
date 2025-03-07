#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "SetupConfig/SetupRobot.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::literals;
namespace fs = std::filesystem;

#define UPDATE_WAIT_TIME 95

// start the executive Loop
class ExecutiveMainLoop : public rclcpp::Node {
public:
  // Setup for all the functions should be done here.
  // Maybe we could code each function to setup on its own.
  // The functions run assuming that the inital first iteration
  // of the loop starts stage by stage with no wait.
  ExecutiveMainLoop() : Node("executive_main_node") loopIsRunning(true), tasksCompleted(false) {
    fs::path stateFilePath = fs::current_path() / "state.csv";
    stateFile.open(stateFilePath, std::ofstream::app);
    // Append header if empty state file
    if (fs::file_size(stateFilePath) == 0) {
      stateFile << "Time,Depth(m),IMU Data,PWM Data\n";
    }
  }

  void depthSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
    //  std::lock_guard<std::mutex> lock(mutex_);
    depth_msg = msg->data;
  }

  void imuSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
    // std::lock_guard<std::mutex> lock(mutex_);
    imu_msg = msg->data;
  }

  // Need to think about this and draw it out. Think about using ROS or not
  // for everything and anything. Need to think about how we should the
  // standard of creating nodes and reading messages. There are multiple
  // implementations, but one of them has to be the best.

  void readInputs() {
    while (loopIsRunning) {
      std::lock_guard<std::mutex> lock(mutex_);
      // Have to check what the msg is saying.
      // Parse msg data. Put the research data into a vector or varibles.
      // IMU data will probabily go into vector.
      // IF needed we can use parameters with ROS if a lot of different types
      // of data. one part of message has to the be imu and the other part has
      // to be the depth. Assuming I have it right. Need to read
      // multithreading with mutex condition and lock pushing to variables. or
      // anyway of setting it
      /*depth;
      vector[]*/
    }
  }

  /* 
   * Get the varaibles and put it into the state file.
   * time, depth, imu, pwm
   * timestamped every 0.1 seconds.
   */
  void updateState() {
    while (loopIsRunning) {
      if(!depth_msg.empty() && !imu_msg.empty() ){
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_WAIT_TIME));
        std::string time = getCurrentTime();
        // TODO: add William's code to add PWM data
        stateFile << time << "," << depth_msg << "," << imu_msg << "," << "N/A";
      }
    }
  }

  void executeDecisionLoop() {
    while (loopIsRunning) {
      // William's code call here
    }
    // if all decisions/tasks are done, make tasksCompleted true;
  }

  // Sends Commands to Thruster Queue
  void sendThrusterCommands() {
    while (loopIsRunning) {
      // send it back to William's code.
    }
  }

  bool ralse;
  }

private:
  std::fstream stateFile;
  std::mutex mutex_;
  std::string depth_msg;
  std::string imu_msg;
  std::vector<float> imu_data;
  float depth;

  bool loopIsRunning;
  bool tasksCompleted;

  std::string getCurrentTime() {
    time_t now = time(0);
    tm *localTime = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
    return std::string(buffer);
  }
};

class SensorsData : public rclcpp::Node {
public:
  SensorsData(std::shared_ptr<ExecutiveMainLoop> mainLoopNode) : Node("sensorsNode") {
    callbackDepth = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto depthOptions = rclcpp::SubscriptionOptions();
    depthOptions.callback_group = callbackDepth;

    callbackIMU = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto imuOptions = rclcpp::SubscriptionOptions();
    imuOptions.callback_group = callbackIMU;

    depth_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "depthSensorData", rclcpp::QoS(5),
      std::bind(&ExecutiveMainLoop::depthSensorCallback, mainLoopNode, std::placeholders::_1), 
      sub1_opt
    );
eturnStatus() { return loopIsRunning; }
  bool returntasksCompleted() { return tasksCompleted; }
  void executeFailCommands() {
    stateFile.close();
    loopIsRunning = f
    // Priority
    // Need to input IMU inilization with ROS.
    imu_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "imuSensorData", rclcpp::QoS(5),
      std::bind(&ExecutiveMainLoop::imuSensorCallback, mainLoopNode,
                std::placeholders::_1),sub2_opt
    );

  }
private:
  rclcpp::CallbackGroup::SharedPtr callbackDepth;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_sensor_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_subscription_;
};

int main(int argc, char *argv[]) {
  // setup time.
  // setup Robot during inilization.
  // std::cout << "Checking" << std::endl;
  rclcpp::init(argc, argv);
  //ExecutiveMainLoop
  auto mainLoopNode = std::make_shared<ExecutiveMainLoop>();
  SetupRobot initStateandConfig = SetupRobot();
  // ExecutiveMainLoop mainLoopNode = ExecutiveMainLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;
  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue
  // system.

  // std::jthread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, mainLoopNode);
  // Creates a new thread for each node. Need to check if it does

  /*
  //auto ReadInputsNode = rclcpp::Node::make_shared("ReadInputsNode");
  executeThreads.add_node(executemainloopThread);
  executeThreads.spin();*/
  // rclcpp::shutdown();
  // rclcpp::spin(ReadInputsNode);

  std::jthread UpdateStateThread(&ExecutiveMainLoop::UpdateState, mainLoopNode);
  std::jthread ExecutiveDecisionLoopThread(&ExecutiveMainLoop::ExecuteDecisionLoop, mainLoopNode);
    // Note: We can join these two threads above and bottom if Rasberry PI
    // really does not like multithreading.
  std::jthread SendThrusterCommandsThread(&ExecutiveMainLoop::SendThrusterCommands, mainLoopNode);
  std::cout << "Here" << std::endl;
  
  rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
  auto sensorNode = std::make_shared<SensorsData>(mainLoopNode);

  SensorsExecutor.add_node(sensorNode);
  SensorsExecutor.spin();

  rclcpp::shutdown();
  std::cout << "Here" << std::endl;
  /*
    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandsThread.join();*/

  return 0;
}