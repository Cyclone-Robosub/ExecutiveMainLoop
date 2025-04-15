/**
 * @file test_executive_loop.cpp
 * @brief Unit test for the ExecutiveLoop node 
 * 
 * This File contains test cases to to verify the function of the SensorData publisher.
 * @author Danny Kwong
 * @date 2025-04-14
 */

#include "test_nodes.hpp"

TEST_F(TestExecutiveLoop, NodeInitialization) {
    EXPECT_EQ(exec_node->get_name(), "executive_main_node");
    EXPECT_TRUE(exec_node->returnStatus());
}

TEST_F(TestExecutiveLoop, CommandSubscription) {
    auto subs = exec_node->get_subscriptions_info_by_topic("python_cltool_topic");
    EXPECT_EQ(subs.size(), 1u);
}

TEST_F(TestExecutiveLoop, EmergencyShutdown) {
    exec_node->executeFailCommands();
    EXPECT_FALSE(exec_node->returnStatus());
    EXPECT_TRUE(exec_node->returntasksCompleted());
}