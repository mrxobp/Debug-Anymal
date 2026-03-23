// Custom safety checker for knee symmetry during oscillation

#pragma once

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalState.hpp>
#include <anymal_motion_control/checks/StateCheckBase.hpp>
#include <message_logger/message_logger.hpp>

namespace anymal_ctrl_oscillator {

// Knee symmetry check for oscillation controllers
// Monitors symmetry between left and right knee joint positions
// Important for detecting mechanical issues or instability in oscillation
class KneeSymmetryCheck : public anymal_motion_control::StateCheckBase {
 public:
  using AD = anymal_description::AnymalDescription;

  // Constructor with configurable asymmetry threshold
  explicit KneeSymmetryCheck(double maxAsymmetry = 0.3) : maxAsymmetry_(maxAsymmetry) {
    MELO_INFO_STREAM("Created knee symmetry check with max asymmetry: " << maxAsymmetry_ << " rad");
  }

  // Check for excessive asymmetry between left and right knee positions
  bool checkState(const anymal_model::AnymalState& state) const override {
    bool isValid = true;

    // Get joint positions from state
    const auto& jointPositions = state.getJointPositions().toImplementation();

    // Check front leg knee symmetry
    const auto lfKneeIdx = AD::mapKeyEnumToKeyId(AD::JointEnum::LF_KFE);
    const auto rfKneeIdx = AD::mapKeyEnumToKeyId(AD::JointEnum::RF_KFE);
    const double frontKneeAsymmetry = std::abs(jointPositions[lfKneeIdx] - jointPositions[rfKneeIdx]);

    if (frontKneeAsymmetry > maxAsymmetry_) {
      MELO_WARN_STREAM("Front knee asymmetry (" << frontKneeAsymmetry << " rad) exceeds limit (" << maxAsymmetry_ << " rad)");
      isValid = false;
    }

    // Check hind leg knee symmetry
    const auto lhKneeIdx = AD::mapKeyEnumToKeyId(AD::JointEnum::LH_KFE);
    const auto rhKneeIdx = AD::mapKeyEnumToKeyId(AD::JointEnum::RH_KFE);
    const double hindKneeAsymmetry = std::abs(jointPositions[lhKneeIdx] - jointPositions[rhKneeIdx]);

    if (hindKneeAsymmetry > maxAsymmetry_) {
      MELO_WARN_STREAM("Hind knee asymmetry (" << hindKneeAsymmetry << " rad) exceeds limit (" << maxAsymmetry_ << " rad)");
      isValid = false;
    }

    return isValid;
  }

 private:
  double maxAsymmetry_;  // Maximum allowed asymmetry between left and right knees (radians)
};

}  // namespace anymal_ctrl_oscillator
