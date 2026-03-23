// ROS integration interface for oscillator controller

#pragma once

#include <geometry_msgs/TwistStamped.h>

#include <anymal_ctrl_oscillator/OscillatorController.hpp>
#include <anymal_model/AnymalState.hpp>
#include <anymal_motion_control_ros/RosNodeInterface.hpp>

#include <cmath>
#include <memory>

namespace anymal_ctrl_oscillator_ros {

// ROS wrapper for OscillatorController
// Handles ROS integration while the base controller implements core logic
class OscillatorControllerRos : public anymal_ctrl_oscillator::OscillatorController, public anymal_motion_control_ros::RosNodeInterface {
 public:
  //! Base controller type
  using OscillatorController = anymal_ctrl_oscillator::OscillatorController;

  // Constructor - initialization happens in create()
  OscillatorControllerRos();

  // Create controller - set up subscribers, parameters and base controller
  bool create() override;

  // Initialize controller with current robot state
  bool doInitialize(const anymal_model::AnymalState state) override;

  // Advance controller one time step
  bool advance(const anymal_model::AnymalState state, anymal_motion_control::Command& command) override;

  // Prepare controller for stopping
  bool preStop() override;

  // Stop the controller
  bool stop() override;

 private:
  // Load parameters from ROS parameter server
  void loadParametersFromROS();

  // Callback for ROS twist messages
  void twistCommandCallback(const geometry_msgs::TwistStamped& msg);

  // Validate twist to ensure it contains no NaN or infinite values
  static bool isValidTwist(const geometry_msgs::Twist& twist);

  //! Shared twist object passed to the controller
  std::shared_ptr<anymal_model::Twist> robotTwist_;

  //! ROS subscriber for twist messages
  ros::Subscriber twistSubscriber_;
};

}  // namespace anymal_ctrl_oscillator_ros
