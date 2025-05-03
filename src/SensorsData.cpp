#include "SensorsDataConfig.h"
#include <iostream>

SensorsDataConfig::SensorsDataConfig(std::shared_ptr<ExecutiveLoop> mainLoopObject)
    : Node("sensorsNode") {
  
  // Create callback groups
  callbackDepthPressure = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackIMU = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackClTool = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  callbackManual = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  
  // Create subscription options
  auto commandOptions = rclcpp::SubscriptionOptions();
  commandOptions.callback_group = callbackClTool;
  
  auto durationOptions = rclcpp::SubscriptionOptions();
  durationOptions.callback_group = callbackClTool;
  
  auto depthPressureOptions = rclcpp::SubscriptionOptions();
  depthPressureOptions.callback_group = callbackDepthPressure;
  
  auto imuOptions = rclcpp::SubscriptionOptions();
  imuOptions.callback_group = callbackIMU;
  
  auto magOptions = rclcpp::SubscriptionOptions();
  magOptions.callback_group = callbackIMU;
  
  auto ManualToggleOptions = rclcpp::SubscriptionOptions();
  ManualToggleOptions.callback_group = callbackManual;
  
  auto ManualOverride = rclcpp::SubscriptionOptions();
  ManualOverride.callback_group = callbackManual;
  
  std::cout << "Creating sensors subscriptions\n";
  
  // Create subscriptions
  depth_pressure_sensor_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "depthPressureSensorData", rclcpp::QoS(5),
      std::bind(&ExecutiveLoop::depthPressureSensorCallback, mainLoopObject, 
                std::placeholders::_1),
      depthPressureOptions);
  
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::QoS(5),
      std::bind(&ExecutiveLoop::imuSensorCallback, mainLoopObject, 
                std::placeholders::_1),
      imuOptions);
  
  mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "mag", rclcpp::QoS(5),
      std::bind(&ExecutiveLoop::magCallback, mainLoopObject, 
                std::placeholders::_1),
      magOptions);
  
  CLTool_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
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
