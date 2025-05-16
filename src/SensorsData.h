#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "ExecutiveLoop.h"

/// @brief SensorsDataConfig class handles ROS2 sensor data subscriptions
class SensorsDataConfig : public rclcpp::Node {
public:
  /// @brief Constructor for SensorsDataConfig
  /// @param mainLoopObject Shared pointer to ExecutiveLoop object for callbacks
  SensorsDataConfig(std::shared_ptr<ExecutiveLoop> mainLoopObject);

private:
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callbackDepthPressure;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::CallbackGroup::SharedPtr callbackClTool;
  rclcpp::CallbackGroup::SharedPtr callbackManual;
  
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_pressure_sensor_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr CLTool_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Control_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Override_sub;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr duration_subscription_;
};
