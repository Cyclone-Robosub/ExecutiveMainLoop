/**
 * @file test_nodes.hpp
 * @brief Test fixtures for ROS 2 nodes
 * 
 * Provides common initialization for:
 * 1. ROS 2 context creation
 * 2. Node instantiation with proper dependency injection
 * 3. Resource cleanup after tests
 *
 * Testing Framework:
 *   Google Test (gtest) for C++ unit testing
 *
 * Fixtures:
 *   TestExecutiveLoop: Combined setup for executive node tests
 *   TestSensorsData: Combined setup for sensor node tests
 *
 * @author Danny Kwong
 * @date April 14, 2025
 */
/*
#pragma once
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "ExecutiveLoop.cpp"  // Include node implementation(s)

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
};*/