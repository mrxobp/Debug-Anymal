// This file registers the controller class with the dynamic class loader system.
// When the controller library is loaded, these macros ensure the OscillatorControllerRos
// class can be discovered and instantiated by the motion control manager at runtime.

#include <dynamic_class_loader/export_macros.hpp>

#include <robot_control/controller/ControllerInterface.hpp>

#include "anymal_ctrl_oscillator_ros/OscillatorControllerRos.hpp"

// Start the registration block for robot_control::ControllerInterface implementations
DYNAMIC_CLASS_LOADER_BEGIN_MANIFEST(robot_control::ControllerInterface)
// Register OscillatorControllerRos as a loadable controller class
DYNAMIC_CLASS_LOADER_EXPORT_CLASS(anymal_ctrl_oscillator_ros::OscillatorControllerRos)
// End the registration block
DYNAMIC_CLASS_LOADER_END_MANIFEST
