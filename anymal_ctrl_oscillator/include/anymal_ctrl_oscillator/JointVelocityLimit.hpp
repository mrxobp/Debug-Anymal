#pragma once

// C++ Standard Library
#include <algorithm>
#include <cmath>

// ANYmal Motion Control
#include <anymal_motion_control/Command.hpp>
#include <anymal_motion_control/limits/CommandLimitBase.hpp>

// ANYmal Description
#include <anymal_description/AnymalDescription.hpp>

// Logging
#include <message_logger/message_logger.hpp>

namespace anymal_ctrl_oscillator {

using AD = anymal_description::AnymalDescription;

/// Example command limit that clamps joint velocities to a maximum value.
class JointVelocityLimit : public anymal_motion_control::CommandLimitBase {
 public:
  /// Constructor.
  /// @param maxAbsoluteVelocity The maximum allowed absolute joint velocity (rad/s).
  explicit JointVelocityLimit(double maxAbsoluteVelocity) : maxAbsoluteVelocity_(std::abs(maxAbsoluteVelocity)) {}

  /// Apply velocity limits to the command.
  /// @param command The motion command object to modify.
  /// @return True if the limit check passes, false only for critical errors.
  bool limitCommand(anymal_motion_control::Command& command) const override {
    // Iterate through all actuators and clamp their joint velocities
    for (const auto& actuatorKey : AD::getActuatorKeys()) {
      auto& actuatorCommand = command.getActuatorCommands()[actuatorKey.getEnum()];
      const double currentVelocity = actuatorCommand.getJointVelocity();
      const double limitedVelocity = std::clamp(currentVelocity, -maxAbsoluteVelocity_, maxAbsoluteVelocity_);

      if (currentVelocity != limitedVelocity) {
        actuatorCommand.setJointVelocity(limitedVelocity);
      }
    }

    // Always return true since applying limits is normal, not an error
    return true;
  }

 private:
  /// The maximum absolute velocity allowed for any joint.
  const double maxAbsoluteVelocity_;
};

}  // namespace anymal_ctrl_oscillator
