// ROS integration implementation for the oscillator controller

// Project header
#include "anymal_ctrl_oscillator_ros/OscillatorControllerRos.hpp"

// ROS headers
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// ANYbotics framework
#include <kindr_ros/kindr_ros.hpp>
#include <message_logger/message_logger.hpp>
#include <param_io/get_param.hpp>
#include <signal_logger/signal_logger.hpp>

namespace anymal_ctrl_oscillator_ros {

// Constructor - initialize the base controller
OscillatorControllerRos::OscillatorControllerRos() : OscillatorController() {
  // No heavy initialization here - that happens in create()
}

// Create the controller
bool OscillatorControllerRos::create() {
  MELO_INFO_STREAM("Creating oscillator controller ROS wrapper");

  // Create the shared twist object
  robotTwist_ = std::make_shared<anymal_model::Twist>();

  // Initialize the base oscillator controller - this handles the core oscillation logic
  if (!OscillatorController::create()) {
    MELO_ERROR("Failed to create oscillator controller");
    return false;
  }

  // Load parameters from ROS parameter server
  loadParametersFromROS();

  // Set up the twist subscriber
  twistSubscriber_ = nodeHandle_.subscribe("/motion_reference/command_twist", 1, &OscillatorControllerRos::twistCommandCallback, this,
                                           ros::TransportHints().tcpNoDelay());

  // Initialize the robotTwist to zero
  robotTwist_->setZero();

  // Add twist variables to signal logger for debugging
  signal_logger::add(*robotTwist_, "robotTwist", "/" + getName());

  return true;
}

// Initialize controller with the current robot state
bool OscillatorControllerRos::doInitialize(const anymal_model::AnymalState state) {
  MELO_INFO_STREAM("Initializing oscillator controller ROS wrapper");
  return OscillatorController::doInitialize(state);
}

// Advance the controller one time step
bool OscillatorControllerRos::advance(const anymal_model::AnymalState state, anymal_motion_control::Command& command) {
  // Forward the twist directly to the base controller and let it implement the core oscillation logic
  OscillatorController::setTwistCommand(*robotTwist_);
  return OscillatorController::advance(state, command);
}

// Prepare controller for stopping
bool OscillatorControllerRos::preStop() {
  MELO_INFO_STREAM("Pre-stopping oscillator controller ROS wrapper");
  return OscillatorController::preStop();
}

// Stop the controller
bool OscillatorControllerRos::stop() {
  MELO_INFO_STREAM("Stopping oscillator controller ROS wrapper");
  return OscillatorController::stop();
}

// Process incoming ROS twist command messages
void OscillatorControllerRos::twistCommandCallback(const geometry_msgs::TwistStamped& msg) {
  // Check if the twist is valid before using it
  if (isValidTwist(msg.twist)) {
    // Convert the message to the robotTwist_ object
    kindr_ros::convertFromRosGeometryMsg(msg.twist.angular, robotTwist_->getRotationalVelocity());
    kindr_ros::convertFromRosGeometryMsg(msg.twist.linear, robotTwist_->getTranslationalVelocity());
  } else {
    // If invalid, log a throttled warning
    MELO_WARN_THROTTLE_STREAM(5.0, "Received invalid twist message with NaN or infinite values");
  }
}

// Validate twist to ensure it contains no NaN or infinite values
bool OscillatorControllerRos::isValidTwist(const geometry_msgs::Twist& twist) {
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) &&
         std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

// Load parameters from ROS parameter server
void OscillatorControllerRos::loadParametersFromROS() {
  using Actuator = anymal_description::AnymalDescription::ActuatorNodeEnum;
  using Params = anymal_ctrl_oscillator::OscillatorController::Parameters;

  // Get current controller parameters (will be used as defaults)
  Params params = dynamic_cast<OscillatorController*>(this)->getParams();

  // Set the parameter namespace
  const std::string paramNs = "/" + getName() + "/";

  // Scalars
  param_io::getParam(nodeHandle_, paramNs + "base_amplitude", params.baseAmplitude);
  param_io::getParam(nodeHandle_, paramNs + "max_amplitude", params.maxAmplitude);

  params.frequency = param_io::param(nodeHandle_, paramNs + "frequency", params.frequency);
  params.minimumVelocityThreshold = param_io::param(nodeHandle_, paramNs + "minimum_velocity_threshold", params.minimumVelocityThreshold);

  // --------------------------------------------------------------------------
  // Command scaling limit (single field: params.commandScale)
  //
  // Priority:
  //  1) New key:  oscillate/limits/max_sagittal_velocity
  //  2) Old key:  kfe_bounce/limits/max_sagittal_velocity
  //  3) Old key:  kfe_pitch/limits/max_sagittal_velocity
  //  4) Default:  keep current params.commandScale
  // --------------------------------------------------------------------------

  // (1) new key
  params.commandScale = param_io::param(nodeHandle_, paramNs + "oscillate/limits/max_sagittal_velocity", params.commandScale);

  // (2) fallback old bounce key
  params.commandScale = param_io::param(nodeHandle_, paramNs + "kfe_bounce/limits/max_sagittal_velocity", params.commandScale);

  // (3) fallback old pitch key
  params.commandScale = param_io::param(nodeHandle_, paramNs + "kfe_pitch/limits/max_sagittal_velocity", params.commandScale);

  // Load PID gains for each joint type
  for (const auto& jointType :
       {std::make_pair(Actuator::HAA, "HAA"), std::make_pair(Actuator::HFE, "HFE"), std::make_pair(Actuator::KFE, "KFE")}) {
    auto& gains = params.gains[jointType.first];
    const std::string gainPath = paramNs + "gains/" + jointType.second + "/";

    bool pGainFound = param_io::getParam(nodeHandle_, gainPath + "p", gains.pGain_);
    bool iGainFound = param_io::getParam(nodeHandle_, gainPath + "i", gains.iGain_);
    bool dGainFound = param_io::getParam(nodeHandle_, gainPath + "d", gains.dGain_);

    if (pGainFound && iGainFound && dGainFound) {
      MELO_INFO_STREAM("Loaded PID gains for " << jointType.second << " joints: P=" << gains.pGain_ << ", I=" << gains.iGain_
                                               << ", D=" << gains.dGain_);
    }
  }

  // Apply updated parameters to the controller
  OscillatorController::setParams(params);
}
}  // namespace anymal_ctrl_oscillator_ros
