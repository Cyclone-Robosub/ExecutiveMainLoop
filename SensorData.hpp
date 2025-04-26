#pragma once

#include "ExecutiveLoop.hpp"

class SensorsData : public rclcpp::Node {
    public:
     SensorsData(std::shared_ptr<ExecutiveLoop> mainLoopObject)
         : Node("sensorsNode") {
       callbackDepth = this->create_callback_group(
           rclcpp::CallbackGroupType::MutuallyExclusive);
       callbackIMU = this->create_callback_group(
           rclcpp::CallbackGroupType::MutuallyExclusive);
   
       auto commandOptions = rclcpp::SubscriptionOptions();
       commandOptions.callback_group = callbackDepth;
       auto depthOptions = rclcpp::SubscriptionOptions();
       depthOptions.callback_group = callbackDepth;
       auto imuOptions = rclcpp::SubscriptionOptions();
       imuOptions.callback_group = callbackIMU;
       std::cout << "Creating sensors subscriptions\n";
       auto magOptions = rclcpp::SubscriptionOptions();
       magOptions.callback_group = callbackIMU;
   
       depth_sensor_subscription_ =
           this->create_subscription<std_msgs::msg::String>(
               "depthSensorData", rclcpp::QoS(5),
               std::bind(&ExecutiveLoop::depthSensorCallback, mainLoopObject,
                         std::placeholders::_1),
               depthOptions);
   
       // Priority
       // Need to input IMU initialization with ROS.
   
       imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
           "imu", rclcpp::QoS(5),
           std::bind(&ExecutiveLoop::imuSensorCallback, mainLoopObject,
                     std::placeholders::_1),
           imuOptions);
       mag_subscription_ =
           this->create_subscription<sensor_msgs::msg::MagneticField>(
               "magtopic", rclcpp::QoS(5),
               std::bind(&ExecutiveLoop::magCallback, mainLoopObject,
                         std::placeholders::_1),
               magOptions);
       pythonCltool_subscription =
           this->create_subscription<std_msgs::msg::Int32MultiArray>(
               "python_cltool_topic", 10,
               std::bind(&ExecutiveLoop::pythonCltoolCallback, mainLoopObject,
                         std::placeholders::_1),
               commandOptions);
     }
   
    private:
     rclcpp::CallbackGroup::SharedPtr callbackDepth;
     rclcpp::CallbackGroup::SharedPtr callbackIMU;
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
         depth_sensor_subscription_;
     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
     rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
         mag_subscription_;
     rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
         pythonCltool_subscription;
   };