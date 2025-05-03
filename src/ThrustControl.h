#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <chrono>
#include <atomic>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "Command_Interpreter.h"

/// @brief PWMController node handles all PWM-related operations
class PWMController : public rclcpp::Node {
public:
  /// @brief Constructor
  /// @param node_name Name of the ROS node
  PWMController(const std::string& node_name = "pwm_controller_node");
  
  /// @brief Initialize the PWM hardware
  bool initialize();
  
  /// @brief Shutdown the PWM controller
  void shutdown();

private:
  // Callback functions
  void manualControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void manualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void pwmArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg);
  
  // Main execution loop
  void executePWMLoop();
  
  // Publishers for status updates
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr current_pwm_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_override_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_array_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr duration_sub_;
  
  // Internal state
  std::atomic<bool> is_manual_enabled_{false};
  std::atomic<bool> is_manual_override_{false};
  std::atomic<bool> is_running_{true};
  std::atomic<bool> is_executing_command_{false};
  std::atomic<bool> allow_duration_sync_{false};
  
  // Command queue and current state
  std::queue<std::pair<pwm_array, std::chrono::milliseconds>> command_queue_;
  pwm_array current_pwm_array_;
  pwm_array pending_pwm_array_;
  std::pair<pwm_array, std::chrono::milliseconds> current_command_;
  
  // Hardware control
  std::unique_ptr<Command_Interpreter_RPi5> command_interpreter_;
  std::vector<PwmPin*> thruster_pins_;
  
  // Thread synchronization
  std::mutex queue_mutex_;
  std::mutex command_mutex_;
  std::mutex sync_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable duration_cv_;
  
  // Execution thread
  std::thread execution_thread_;
  
  // Utility functions
  void setNeutralPWM();
  void overrideCurrentCommand();
  void executeCommand(const std::pair<pwm_array, std::chrono::milliseconds>& command);
  void publishStatus(const std::string& status);
  void publishCurrentPWM();
  
  // Configuration constants
  static constexpr int NEUTRAL_PWM = 1500;
  static constexpr int ZERO_PWM = 0;
  static constexpr int NUM_THRUSTERS = 8;
  static constexpr int64_t INFINITE_DURATION = 9999999999;
};
