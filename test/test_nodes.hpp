/**
 * @file test_nodes.hpp
 * @brief Test fixtures for ROS 2 nodes
 * 
 * Provides common initialization and teardown logic for unit tests
 * involving ExecutiveLoop and SensorsData nodes.
 * 
 * Key Responsibilities:
 * - ROS 2 context setup and shutdown
 * - Node instantiation and dependency injection
 * 
 * Fixtures:
 *   - TestExecutiveLoop: Tests core functionality of ExecutiveLoop
 *   - TestSensorsData: Tests sensor callback and message handling
 * 
 * @author Danny Kwong
 * @date April 14, 2025
 */

 #pragma once

 #include <gtest/gtest.h>
 #include <rclcpp/rclcpp.hpp>
 #include "ExecutiveLoop.hpp"
 #include "SensorData.hpp"
 
 class TestExecutiveLoop : public ::testing::Test {
 protected:
     void SetUp() override {
         rclcpp::init(0, nullptr);
         exec_node = std::make_shared<ExecutiveLoop>();
         sensor_node = std::make_shared<SensorsData>(exec_node);
     }
 
     void TearDown() override {
         sensor_node.reset();
         exec_node.reset();
         rclcpp::shutdown();
     }
 
     std::shared_ptr<ExecutiveLoop> exec_node;
     std::shared_ptr<SensorsData> sensor_node;
 };
 
 class TestSensorsData : public ::testing::Test {
 protected:
     void SetUp() override {
         rclcpp::init(0, nullptr);
         exec_node = std::make_shared<ExecutiveLoop>();
         sensor_node = std::make_shared<SensorsData>(exec_node);
     }
 
     void TearDown() override {
         sensor_node.reset();
         exec_node.reset();
         rclcpp::shutdown();
     }
 
     std::shared_ptr<ExecutiveLoop> exec_node;
     std::shared_ptr<SensorsData> sensor_node;
 };