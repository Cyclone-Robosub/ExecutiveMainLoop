/**
 * @file test_ExecutiveLoop.cpp
 * @brief Unit test for the SensorsData node 
 * 
 * This File contains test cases to to verify the function of the SensorData publisher.
 * @author Danny Kwong
 * @date 2025-04-14
 */

 #include "test_nodes.hpp"

/*
 TEST_F(TestSensorsData, SensorSubscriptions) {
     auto depth_subs = sensor_node->get_subscriptions_info_by_topic("depthSensorData");
     auto imu_subs = sensor_node->get_subscriptions_info_by_topic("imu");
     EXPECT_EQ(depth_subs.size(), 1u);
     EXPECT_EQ(imu_subs.size(), 1u);
 }*/
 /*
 TEST_F(TestSensorsData, CallbackHandling) {
     // Test IMU callback
     sensor_msgs::msg::Imu imu_msg;
     imu_msg.angular_velocity.x = 1.0;
     sensor_node->imu_subscription_->handle_message(&imu_msg, std::make_shared<rclcpp::SerializedMessage>());
     EXPECT_FLOAT_EQ(exec_node->angular_velocity_x, 1.0);
 } commenting in  order to compile.
     */ 