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


// start the executive Loop
class ExecutiveMainLoop : public rclcpp::Node {
  public:
    // Setup for all the functions should be done here.
    // Maybe we could code each function to setup on its own.
    // The functions run assuming that the inital first iteration
    // of the loop starts stage by stage with no wait.
    ExecutiveMainLoop() : Node("executive_main_node") {

      fs::path currentPath = fs::current_path();
      currentPath += "state.csv";

      if (!std::filesystem::exists(currentPath)) {
        outStateFile.open(currentPath, std::ofstream::app);

        // Append this for every new file.
        outStateFile << "Time,Depth(m),IMU Data, PWM Data\n";
      }
    }
    void depthSensorCallback(const std_msgs::msg::String::SharedPtr msg)
         {
        //  std::lock_guard<std::mutex> lock(mutex_);
      depth_msg = msg->data;
    }

    void imuSensorCallback(const std_msgs::msg::String::SharedPtr msg)
         {
      // std::lock_guard<std::mutex> lock(mutex_);
      imu_msg = msg->data;
    }

    // Need to think about this and draw it out. Think about using ROS or not
    // for everything and anything. Need to think about how we should the
    // standard of creating nodes and reading messages. There are multiple
    // implementations, but one of them has to be the best.

    void ReadInputs() {

      while (status) {

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

    void UpdateState() {
      //get a notification here

      while (status) {
        // Get the varaibles and put it into the state file.Timestampted for
        // every 0.1seconds.
        if(!depth_msg.empty() && !imu_msg.empty() ){
        std::this_thread::sleep_for(std::chrono::milliseconds(95));
        outStateFile << "," << depth_msg << "," << imu_msg;
        }
        // add Time element
        // Need to see William's code to put PWM here in the status file.
      }
    }

    void ExecuteDecisionLoop() {
      while (status) {
        // William's code call here
      }
      // if all decisions/tasks are done make TasksCompleted true;
    }

    // Sends Commands to Thruster Queue
    void SendThrusterCommands() {
      while (status) {
        //send it back to William's code.
      }
    }

    bool returnStatus() { return status; }
    bool returnTasksCompleted() { return TasksCompleted; }
    void ExecuteFailCommands() {
      outStateFile.close();
      status = false;
    }

  private:
    std::fstream outStateFile;

    float depth;
    std::vector<float> imu_data;
    std::mutex mutex_;
    std::string depth_msg;
    std::string imu_msg;

    bool status = true;
    bool TasksCompleted = false;

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
  SensorsData(std::shared_ptr<ExecutiveMainLoop> mainLoop) : Node("sensorsNode") {
    callbackDepth = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackIMU = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callbackDepth;
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callbackIMU;

    depth_sensor_Subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthSensorData", rclcpp::QoS(5),
            std::bind(&ExecutiveMainLoop::depthSensorCallback,mainLoop,
                      std::placeholders::_1), sub1_opt);

    // Priority
    // Need to input IMU inilization with ROS.
    imu_sensor_Subscription_ = this->create_subscription<std_msgs::msg::String>(
        "imuSensorData", rclcpp::QoS(5),
        std::bind(&ExecutiveMainLoop::imuSensorCallback, mainLoop,
                  std::placeholders::_1),sub2_opt);

  }
  private:
  rclcpp::CallbackGroup::SharedPtr callbackDepth;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        depth_sensor_Subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        imu_sensor_Subscription_;
};


  int main(int argc, char *argv[]) {
    // setup time.
    //  setup Robot during inilization.
    // std::cout << "Checking" << std::endl;
    rclcpp::init(argc, argv);
    auto mainLoop = std::make_shared<ExecutiveMainLoop>();
    SetupRobot initStateandConfig = SetupRobot();
    // ExecutiveMainLoop mainLoop = ExecutiveMainLoop(argc, argv);
    // records false if run has not completed yet.
    bool runStatus = false;
    // these threads functions will have loops that go on for ever
    // these functions will have wait functions just in case with a queue
    // system.

   // std::jthread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, mainLoop);
    // Creates a new thread for each node. Need to check if it does

    /*
    //auto ReadInputsNode = rclcpp::Node::make_shared("ReadInputsNode");
    executeThreads.add_node(executemainloopThread);
    executeThreads.spin();*/
    // rclcpp::shutdown();
    // rclcpp::spin(ReadInputsNode);

    std::jthread UpdateStateThread(&ExecutiveMainLoop::UpdateState, mainLoop);
    std::jthread ExecutiveDecisionLoopThread(
        &ExecutiveMainLoop::ExecuteDecisionLoop, mainLoop);
    // Note: We can join these two threads above and bottom if Rasberry PI
    // really does not like multithreading.
    std::jthread SendThrusterCommandsThread(
        &ExecutiveMainLoop::SendThrusterCommands, mainLoop);
    std::cout << "Here" << std::endl;
    rclcpp::executors::MultiThreadedExecutor SensorsExecutor;
      auto sensorNode = std::make_shared<SensorsData>(mainLoop);
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