#pragma once

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
#include <fstream>
#include <iostream>

#include "Command_Interpreter.h"
#include "SetupConfig/SetupRobot.cpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

using namespace std::literals;
namespace fs = std::filesystem;

#define UPDATE_WAIT_TIME 5
#define IMU_SENSOR_WAIT_TIME 2
#define NULL_SENSOR_VALUE -320000
#define FAULTY_SENSOR_VALUE -40404

class ExecutiveLoop : public rclcpp::Node {
public:
  ExecutiveLoop(
		  std::unique_ptr<Command_Interpreter_RPi5> commandInterpreter_ptr, 
		  std::shared_ptr<std::pair<pwm_array, 
		  std::chrono::milliseconds>> currentPWMandDuration_ptr, 
		  std::ofstream& stateFile, 
		  std::ostream& output, 
		  std::ostream& error);

  void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg);

  void imuSensorCallback(const sensor_msgs::msg::Imu &msg);
  void magCallback(const sensor_msgs::msg::MagneticField &msg);

  void PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg);

  void updateState();
  void executeDecisionLoop();

  void sendThrusterCommand();

  bool returnStatus();
  bool returntasksCompleted();
  void executeFailCommands();

private:
  //TODO: there are going to be a lot of unused variables, please remove in the future.
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

  // The current PWM and duration ptr will and should always have a value regardless of what Executive DecisionLoop or
  // Send Thrusters want. However, SendThrusters can "finish" a current PWM and duration and will say that it wants a
  // new command, but it can be the same current PWM if ExecutiveDecision decides so. Executive Decision (on its own
  // thread) will see that SendThrusters is not running a command and give it a new current PWM. This is made so that
  // Executive Decision has the chance to give PWM a new Command if the current one is a timedPWM. In Later uses, the
  // State file should use the current PWM that the Send Thruster is using.
  std::shared_ptr<std::pair<pwm_array, std::chrono::milliseconds>>
      currentPWMandDuration_ptr;
  // bool isQueuePWMEmpty = true;
  std::ofstream& stateFile;
  std::ostream& output;
  std::ostream& error;
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

  std::string getCurrentDateTime();
  void clearQueue();
  void override();
};
