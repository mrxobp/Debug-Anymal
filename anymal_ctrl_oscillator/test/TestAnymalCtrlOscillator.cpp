#include <gtest/gtest.h>

#include "anymal_ctrl_oscillator/OscillatorController.hpp"

// Simple dummy test for the controller
TEST(TestAnymalCtrlOscillator, dummyTest) {  // NOLINT
  ASSERT_TRUE(true);
}

// Basic instantiation test
TEST(TestAnymalCtrlOscillator, instantiationTest) {  // NOLINT
  // Create a controller instance
  anymal_ctrl_oscillator::OscillatorController controller;

  // Check that it was created successfully
  ASSERT_NE(&controller, nullptr);
}
