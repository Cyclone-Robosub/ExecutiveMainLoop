#pragma once

#include "ExecutiveLoop.hpp"
#include "rclcpp/rclcpp.hpp"

class SensorsDataConfig : public rclcpp::Node {
    public:
      SensorsDataConfig(std::shared_ptr<ExecutiveLoop> mainLoopObject);
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