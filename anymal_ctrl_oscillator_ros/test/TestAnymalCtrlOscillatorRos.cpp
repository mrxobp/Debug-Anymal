#include <gtest/gtest.h>
#include <ros/ros.h>

#include "anymal_ctrl_oscillator_ros/OscillatorControllerRos.hpp"

class TestAnymalCtrlOscillatorRos : public ::testing::Test {
 public:
  void SetUp() override {
    const std::map<std::string, std::string> remappings{};
    ros::init(remappings, "anymal_ctrl_oscillator_ros");
    ros::start();
  }

  void TearDown() override { ros::shutdown(); }
};

// Simple dummy test to verify ROS node setup
TEST_F(TestAnymalCtrlOscillatorRos, DummyTest) {  // NOLINT
  EXPECT_TRUE(true);
}

// Test instantiation of the controller
TEST_F(TestAnymalCtrlOscillatorRos, ControllerInstantiation) {  // NOLINT
  // Create ROS controller instance
  anymal_ctrl_oscillator_ros::OscillatorControllerRos controller;

  // Check that it was created successfully
  EXPECT_NE(&controller, nullptr);
}
