#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "SetupConfig/Setup.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::literals;
namespace fs = std::filesystem;
// start the executive Loop
class ExecutiveMainLoop : public rclcpp::Node {
 public:
  // Setup for all the functions should be done here.
  // Maybe we could code each function to setup on its own.
  // The functions run assuming that the inital first iteration
  // of the loop starts stage by stage with no wait.
  ExecutiveMainLoop(int argc, char* argv[]) : Node("executive_main_loop") {
    fs::path currentPath = fs::current_path();
    if (!std::filesystem::exists(currentPath + "state.csv")) {
      outStateFile.open(something + "state.csv", std::ofstream::app);
      // Append this for every new file.
      outStateFile << "Time,Depth(m),IMU Data, PWM Data\n";
    }
    depth_sensor_Subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthSensorData", 5,
            std::bind(&ExecutiveMainLoop::ReadInputs, this,
                      std::placeholders::_1));

    // Priority
    imu_sensor_Subscription_ = this->create_subscription<std_msgs::msg::String>(
        "imuSensorData", 5,
        std::bind(&ExecutiveMainLoop::ReadInputs, this, std::placeholders::_1));
  }

  // Need to think about this and draw it out. Think about using ROS or not for
  // everything and anything. Need to think about how we should the standard of
  // creating nodes and reading messages. There are multiple implementations,
  // but one of them has to be the best.
  void ReadInputs(const std_msgs::msg::String::SharedPtr msg) {
    while (status) {
      //Have to check what the msg is saying.
      // Parse msg data. Put the research data into a vector or varibles.
      // IMU data will probabily go into vector.
      // IF needed we can use parameters with ROS if a lot of different types of
      // data. one part of message has to the be imu and the other part has to
      // be the depth. Assuming I have it right. Need to read multithreading
      // with mutex condition and lock pushing to variables. or anyway of
      // setting it
     /*depth;
     vector[]*/
    }
  }

  void UpdateState() {
    while (status) {
      // Get the varaibles and put it into the state file.Timestampted for every
      // 0.1seconds.
      std::this_thread::sleep_for(std::chrono::milliseconds(95));
      outStateFile << Time << "," << depth << "," << imu_data << std::endl;
      //Need to see William's code to put PWM here.
    }
  }

  void ExecuteDecisionLoop() {
    while (status) {
    }
    // if all decisions/tasks are done make TasksCompleted true;
  }

  // Sends Commands to Thruster Queue
  void SendThrusterCommands() {
    while (status) {
    }
  }

  bool returnStatus() { return status; }
  bool returnTasksCompleted() { return TasksCompleted; }
  void ExecuteFailCommands() {
    outFile.close();
    status = false;
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      depth_sensor_Subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      imu_sensor_Subscription_;
  float depth;
  std::vector<float> imu_data;
  bool status = true;
  std::fstream outStateFile;
  bool TasksCompleted = false;
  std::string getCurrentDateTime() {
    time_t now = time(0);
    tm* localTime = localtime(&now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
    return std::string(buffer);
  }
};

int main(int argc, char* argv[]) {
  // setup time.
  //  setup Robot during inilization.
  // std::cout << "Checking" << std::endl;
  rclcpp::init(argc, argv);
  SetupRobot initStateandConfig = SetupRobot();
  ExecutiveMainLoop mainLoop = ExecutiveMainLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;

  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue system.

  // std::thread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, &mainLoop);

  auto ReadInputsNode = rclcpp::Node::make_shared("ReadInputsNode");
  rclcpp::spin(ReadInputsNode);

  std::jthread UpdateStateThread(&ExecutiveMainLoop::UpdateState, &mainLoop);
  std::jthread ExecutiveDecisionLoopThread(
      &ExecutiveMainLoop::ExecuteDecisionLoop, &mainLoop);
      //Note: We can join these two threads above and bottom if Rasberry PI really does not like multithreading.
  std::jthread SendThrusterCommandsThread(
      &ExecutiveMainLoop::SendThrusterCommands, &mainLoop);

  

  /*
    ReadInputsThread.join();
   UpdateStateThread.join();
   ExecutiveDecisionLoopThread.join();
   SendThrusterCommandsThread.join();*/

  return 0;
}