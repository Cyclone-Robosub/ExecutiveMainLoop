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
  ExecutiveMainLoop(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    research_sensor_Subscription =
        this->create_subscription<std__msgs::msg::String>(
            "researchSensorsData", 5,
            std::bind(&ExecutiveMainLoop::ReadInputs, this,
                      std::placeholders::_1, "researchSensorsData"));
    imu_sensor_Subscription = this->create_subscription<std__msgs::msg::String>(
        "", 5, //Need to make IMU topic name. and put it in the bind function as well.
        std::bind(&ExecutiveMainLoop::ReadInputs, this, std::placeholders::_1));
  }
  // Need to think about this and draw it out. Think about using ROS or not for
  // everything and anything. Need to think about how we should the standard of
  // creating nodes and reading messages. There are multiple implementations,
  // but one of them has to be the best.
  void ReadInputs(const std_msgs::String::ConstPtr& msg) : Node("ReadInputsNode") {
    while (status) {
      //Parse msg data. Put the research data into a vector or varibles.
      //IMU data will probabily go into varabiles.
      //IF needed we can use parameters with ROS if a lot of different types of data.
    }
  }

  void UpdateState() {
    while (status) {
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
  void ExecuteFailCommands() { status = false; }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      research_sensor_Subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      imu_sensor_Subscription_;
  std::vector<float> research_data;
  bool status = true;
  bool TasksCompleted = false;
  
};
int main(int argc, char* argv[]) {
  // setup Robot during inilization.
  Setup initStateandConfig = Setup();
  ExecutiveMainLoop mainLoop(argc, argv);
  // records false if run has not completed yet.
  bool runStatus = false;

  // these threads functions will have loops that go on for ever
  // these functions will have wait functions just in case with a queue system.
  // The main thread will continue doing other things to ensure these threads
  // can keep going.
  /*
  Replace this code with ros2.spin() as soon as possible.
  */
  std::jthread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, &mainLoop);
  std::jthread UpdateStateThread(&ExecutiveMainLoop::UpdateState, &mainLoop);
  std::jthread ExecutiveDecisionLoopThread(
      &ExecutiveMainLoop::ExecuteDecisionLoop, &mainLoop);
  std::jthread SendThrusterCommandsThread(
      &ExecutiveMainLoop::SendThrusterCommands, &mainLoop);
  while (mainLoop.TasksCompleted() == false && mainLoop.status == true) {
    // wait for 5 seconds.
  }

  ReadInputsThread.join();
  UpdateStateThread.join();
  ExecutiveDecisionLoopThread.join();
  SendThrusterCommandsThread.join();

  return 0;
}