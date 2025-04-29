/**
 * @file test_sensor_data.cpp
 * @brief Unit test for the SensorsData node 
 * 
 * This File contains test cases to to verify the function of the SensorData publisher.
 * @author Danny Kwong
 * @date 2025-04-14
 */

 #include "test_nodes.hpp"

 TEST_F(TestSensorsData, SensorSubscriptions) {
     auto depth_subs = sensor_node->get_subscriptions_info_by_topic("depthSensorData");
     auto imu_subs = sensor_node->get_subscriptions_info_by_topic("imu");
     EXPECT_EQ(depth_subs.size(), 1u);
     EXPECT_EQ(imu_subs.size(), 1u);
 }
 
 TEST_F(TestSensorsData, CallbackHandlingIMU) {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();

    // Set the test values
    imu_msg->angular_velocity.x = 1.0;
    imu_msg->angular_velocity.y = 2.0;
    imu_msg->angular_velocity.z = 3.0;
    imu_msg->linear_acceleration.x = 4.0;
    imu_msg->linear_acceleration.y = 5.0;
    imu_msg->linear_acceleration.z = 6.0;

    // Subscription with proper callback
    auto imu_subscription_ = sensor_node->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::QoS(5),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            exec_node->imuSensorCallback(*msg);
        });

    // Now, simulate publishing the message.
    // (You need to actually publish to trigger the callback if you want the full pipeline.)

    // Or, for test simplicity: call the callback directly
    exec_node->imuSensorCallback(*imu_msg);

    // Expect that exec_node internal values were set properly
    EXPECT_FLOAT_EQ(exec_node->getAngularVelocityX(), 1.0);
    EXPECT_FLOAT_EQ(exec_node->getAngularVelocityY(), 2.0);
    EXPECT_FLOAT_EQ(exec_node->getAngularVelocityZ(), 3.0);
    EXPECT_FLOAT_EQ(exec_node->getLinearAccelerationX(), 4.0);
    EXPECT_FLOAT_EQ(exec_node->getLinearAccelerationY(), 5.0);
    EXPECT_FLOAT_EQ(exec_node->getLinearAccelerationZ(), 6.0);
}

TEST_F(TestSensorsData, CallbackHandlingMagneticField) {
    auto mag_msg = std::make_shared<sensor_msgs::msg::MagneticField>();

    // Set test values
    mag_msg->magnetic_field.x = 0.1;
    mag_msg->magnetic_field.y = 0.2;
    mag_msg->magnetic_field.z = 0.3;

    // Create subscription with proper callback
    auto mag_subscription_ = sensor_node->create_subscription<sensor_msgs::msg::MagneticField>(
        "magnetic_field", rclcpp::QoS(5),
        [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
            exec_node->magCallback(*msg);
        });

    // Directly invoke the callback for testing
    exec_node->magCallback(*mag_msg);

    // Validate that exec_node internal values were set correctly
    EXPECT_FLOAT_EQ(exec_node->getMagFieldX(), 0.1);
    EXPECT_FLOAT_EQ(exec_node->getMagFieldY(), 0.2);
    EXPECT_FLOAT_EQ(exec_node->getMagFieldZ(), 0.3);
}

TEST_F(TestSensorsData, CallbackHandlingDepthSensor) {
    auto depth_msg = std::make_shared<std_msgs::msg::String>();
    depth_msg->data = "42.5";

    // Create subscription with proper callback
    auto depth_subscription_ = sensor_node->create_subscription<std_msgs::msg::String>(
        "depth", rclcpp::QoS(5),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            exec_node->depthSensorCallback(msg);
        });

    // Directly invoke the callback for testing
    exec_node->depthSensorCallback(depth_msg);

    // Validate that exec_node internal value was set correctly
    EXPECT_EQ(exec_node->getDepth(), "42.5");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}