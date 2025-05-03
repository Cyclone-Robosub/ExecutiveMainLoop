#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "Command_Interpreter.h"

#define UPDATE_WAIT_TIME 5
#define IMU_SENSOR_WAIT_TIME 2
#define NULL_SENSOR_VALUE -320000
#define FAULTY_SENSOR_VALUE -40404

/// @brief Executive Loop class that manages the main control loop of the robot
class ExecutiveLoop : public rclcpp::Node {
public:
  /// @brief Constructor for ExecutiveLoop
  /// @param commandInterpreter_ptr Unique pointer to command interpreter
  /// @param currentPWMandDuration_ptr Shared pointer to current PWM and duration
  /// @param stateFile Output file stream for state logging
  /// @param output Output stream for general logging
  /// @param error Error stream for error logging
  ExecutiveLoop(std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr, 
                std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>> currentPWMandDuration_ptr, 
                std::ofstream& stateFile, std::ostream& output, std::ostream& error);

  // Callback functions
  void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ManualOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg);
  void imuSensorCallback(const sensor_msgs::msg::Imu &msg);
  void magCallback(const sensor_msgs::msg::MagneticField &msg);
  void PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg);

  // Main loop functions
  void updateState();
  void executeDecisionLoop();
  void sendThrusterCommand();
  
  // Utility functions
  bool returnStatus();
  bool returntasksCompleted();
  void executeFailCommands();

private:
  // Control flags
  bool isManualEnabled = false;
  bool isManualOverride = false;
  bool isRunningThrusterCommand = false;
  bool isCurrentCommandTimedPWM = false;
  bool AllowDurationSync = false;
  bool loopIsRunning = true;  // Changed to true for initialization
  bool tasksCompleted = true;
  
  // Mutexes
  std::mutex thruster_mutex;
  std::mutex array_duration_sync_mutex;
  std::mutex Manual_Mutex;
  std::mutex Manual_Override_mutex;
  std::mutex sensor_mutex;
  std::mutex Queue_pwm_mutex;
  std::mutex imu_mutex;
  std::mutex ThrusterCommand_mutex;
  std::mutex current_PWM_duration_mutex;
  
  // Condition variables
  std::condition_variable SendToDuration_change;
  std::condition_variable PWM_cond_change;
  std::condition_variable Thruster_cond_change;
  std::condition_variable Change_Manual;
  
  // Queue management
  unsigned int sizeQueue = 0;
  std::queue<std::pair<pwm_array, std::chrono::milliseconds>> ManualPWMQueue;
  
  // Sensor data
  float angular_velocity_x = NULL_SENSOR_VALUE;
  float angular_velocity_y = NULL_SENSOR_VALUE;
  float angular_velocity_z = NULL_SENSOR_VALUE;
  float linear_acceleration_x = NULL_SENSOR_VALUE;
  float linear_acceleration_y = NULL_SENSOR_VALUE;
  float linear_acceleration_z = NULL_SENSOR_VALUE;
  float mag_field_x = NULL_SENSOR_VALUE;
  float mag_field_y = NULL_SENSOR_VALUE;
  float mag_field_z = NULL_SENSOR_VALUE;
  float depth = NULL_SENSOR_VALUE;
  float pressure = NULL_SENSOR_VALUE;
  
  // Command and control structures
  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr;
  std::vector<PwmPin *> thrusterPins;
  std::vector<DigitalPin *> digitalPins;
  pwm_array our_pwm_array;
  pwm_array given_array;
  std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>> currentPWMandDuration_ptr;
  
  // Files and streams
  std::ofstream& stateFile;
  std::ostream& output;
  std::ostream& error;
  
  // Status strings
  std::string depth_pressure_msg = "Depth Sensor Not Started Yet";
  std::string imu_msg;
  std::string userinput;
  std::string typeOfExecute;
  
  // Data containers
  std::vector<float> imu_data;
  int duration_int_pwm;
  
  // Private utility functions
  std::string getCurrentDateTime();
  void override();
};
