// Controller implementation for oscillating ANYmal's knee joints

// Project headers
#include "anymal_ctrl_oscillator/OscillatorController.hpp"
#include "anymal_ctrl_oscillator/JointVelocityLimit.hpp"
#include "anymal_ctrl_oscillator/KneeSymmetryCheck.hpp"

// External dependencies
#include <algorithm>
#include <cmath>

// ANYmal framework
#include <anymal_description/AnymalTopology.hpp>
#include <message_logger/message_logger.hpp>
#include <signal_logger/signal_logger.hpp>
// State checkers
#include <anymal_motion_control/checks/BaseStateCheck.hpp>
#include <anymal_motion_control/checks/JointStateCheck.hpp>
#include <anymal_motion_control/checks/StateStatusCheck.hpp>

namespace {
constexpr double PI = 3.14159265358979323846;
}

namespace anymal_ctrl_oscillator {

using JointEnum = anymal_description::AnymalTopology::JointEnum;

// Helper function to convert joint enum to integer index
inline constexpr Eigen::Index jointIdx(JointEnum joint) noexcept {
  return static_cast<Eigen::Index>(joint);
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                          CREATION OF CONTROLLER
//==============================================================================
bool OscillatorController::create() {
  MELO_INFO_STREAM("Creating oscillator controller");

  // Register available operation modes for each reference type
  setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::TWIST, {"oscillate"});

  // Add state checkers for safety
  // 1. Check state estimator status
  // getInitialStateChecker().addStateCheck("stateStatus",
  //                                       std::make_shared<anymal_motion_control::StateStatusCheck>(std::vector<anymal_model::StateStatus>(
  //                                           {anymal_model::StateStatus::STATUS_OK, anymal_model::StateStatus::STATUS_UNINITIALIZED})));

  // 2. Joint state check for finite values and reasonable limits
  getInitialStateChecker().addStateCheck("jointState", std::make_shared<anymal_motion_control::JointStateCheck>());

  // 3. Basic orientation check using the standard framework check
  // getStateChecker().addStateCheck("baseState", std::make_shared<anymal_motion_control::BaseStateCheck>(10.0, 10.0, 0.52));

  // Add periodic state checkers that run during controller execution
  // getStateChecker().addStateCheck("jointState", std::make_shared<anymal_motion_control::JointStateCheck>());

  // 4. Custom symmetry check to monitor left-right knee symmetry during oscillation
  // Demonstrates how to implement and use a specialized check
  const double maxKneeAsymmetry = 0.3;  // rad - ensure left-right symmetry
  getStateChecker().addStateCheck("kneeSymmetry", std::make_shared<KneeSymmetryCheck>(maxKneeAsymmetry));

  // Add command limiters for safety
  // Note: AnymalController already has a default ActuatorCommandLimit that performs basic checks,
  // but you can add custom limiters like this one for additional specialized constraints
  const double maxJointVelocityLimit = 0.8;  // rad/s
  getCommandLimiter().addCommandLimit("jointVelocityLimit", std::make_shared<JointVelocityLimit>(maxJointVelocityLimit));

  addVariablesToLog();
  return true;
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                          INITIALIZATION OF CONTROLLER
//==============================================================================
bool OscillatorController::doInitialize(const anymal_model::AnymalState state) {
  currentJointPositions_ = state.getJointPositions().vector();
  MELO_INFO_STREAM("INITIALISATION Joint positions: " << currentJointPositions_.transpose());
  MELO_INFO_STREAM("doInitialize oscillator controller");

  // === Reset timing and mode ===
  oscillationTime_ = 0.0;
  lastUpdateTime_ = Clock::now();
  currentMode_ = OperationMode::OSCILLATE;
  reset_ = 0.0;
  first_time = 1.0;

  // === The starting position must match JointsOscillation() at (s = sin(phase) = 0) ===
  gotoDefaultStartPos_.resize(desJointPositions_.size());
  gotoDefaultTargetPos_.resize(desJointPositions_.size());

  static constexpr std::array<double, 12> direction = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  static constexpr std::array<double, 12> bias = {0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8};
  static constexpr std::array<double, 12> scale = {0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0};
  for (int i = 0; i < 12; ++i) {
    // When s = 0 it means that shaped = (0 + bias[i]) * direction[i] * scale[i]
    gotoDefaultTargetPos_[i] = bias[i] * direction[i] * scale[i];
  }

  //============== Creation of csv file to log positions =======================
  if (logEnabled_) {
    logFile_.open(logPath_, std::ios::out | std::ios::trunc);
    if (!logFile_.is_open()) {
      MELO_WARN_STREAM("Could not open log file: " << logPath_);
      logEnabled_ = false;
    } else {
      logFile_ << "t,dt,chirp";
      for (int i = 0; i < 12; ++i) logFile_ << ",q_meas_" << i;
      for (int i = 0; i < 12; ++i) logFile_ << ",q_des_" << i;
      logFile_ << "\n";
    }
  }
  return true;
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                              ADVANCE LOOP
//==============================================================================
bool OscillatorController::advance(const anymal_model::AnymalState state, anymal_motion_control::Command& command) {
  const double forwardVelocity = twistCommand_.getTranslationalVelocity().x();

  //========== To log the position of joints in a csv file ====================
  Eigen::VectorXd qMeasByActuatorId;
  qMeasByActuatorId.resize(desJointPositions_.size());
  qMeasByActuatorId.setZero();
  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const int actuatorId = actuatorKey.getId();
    if (actuatorId < 0 || actuatorId >= qMeasByActuatorId.size()) continue;
    qMeasByActuatorId[actuatorId] = currentJointPositions_[actuatorId];
  }

  // === Reset oscillation time when GoToDefault is done, oscillating will begin ===
  if (first_time == 1.0) {
    MELO_INFO_STREAM("=== FIRST TIME FOR ADVANCE ===")
    centerPositions_ = currentJointPositions_;
    oscillationTime_ = 0.0;
    lastUpdateTime_ = Clock::now();
    startGoToDefault(4.0);
    first_time = 0.0;
    MELO_INFO_STREAM("=== Joint position vector size: " << currentJointPositions_.size() << " ===");
    MELO_INFO_STREAM("=== ActuatorKeys (enum,id) list ===");
    for (const auto& actuatorKey : AD::getActuatorKeys()) {
      MELO_INFO_STREAM("actuator enum=" << static_cast<int>(actuatorKey.getEnum()) << " id=" << actuatorKey.getId());
    }
    MELO_INFO_STREAM("=== First 12 entries of state.getJointPositions().vector() ===");
    std::ostringstream oss;
    for (int i = 0; i < 12 && i < currentJointPositions_.size(); ++i) {
      oss << " [" << i << "]=" << currentJointPositions_[i];
    }
    MELO_INFO_STREAM(oss.str());
  } else {
  }

  // === Reset oscillation time when reset controller, oscillating will begin ===
  double dt = 0.0;
  if (reset_ == 1.0) {
    const auto currentTime = Clock::now();
    dt = 0;
    oscillationTime_ = 0;
    lastUpdateTime_ = currentTime;
    reset_ = 0.0;
  } else {
    const auto currentTime = Clock::now();
    dt = std::chrono::duration_cast<Duration>(currentTime - lastUpdateTime_).count();
    oscillationTime_ += dt;
    lastUpdateTime_ = currentTime;
  }

  // === Handle initial stabilization period - freeze joints ===
  if (oscillationTime_ < params_.stabilizationPeriod) {
    for (const auto& actuatorKey : AD::getActuatorKeys()) {
      command.getActuatorCommands()[actuatorKey.getEnum()].setMode(series_elastic_actuator::SeActuatorCommand::MODE_FREEZE);
    }
    return true;
  }

  // === Apply commands to joints ===
  setJointsCommand(dt);
  applyControlCommand(command);

  // === Register in CSV file: time, dt, chirp, qMeasByActuatorId and desJointPositions_ ===
  if (logEnabled_) {
    if ((logCounter_++ % logEveryN_) == 0) {
      const double t = oscillationTime_;

      logFile_ << t << "," << dt << "," << chirpLast_;

      for (int i = 0; i < 12; ++i) {
        logFile_ << "," << qMeasByActuatorId[i];
      }

      for (int i = 0; i < 12; ++i) {
        logFile_ << "," << desJointPositions_[i];
      }

      logFile_ << "\n";

      if ((logCounter_ % (logEveryN_ * 200)) == 0) {
        logFile_.flush();
      }
    }
  }
  return true;
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                                APPLY COMMAND
//==============================================================================
void OscillatorController::setJointsCommand(double dt) {
  // GoToDefault
  if (gotoDefaultActive_) {
    GoToDefault(dt);
    return;
  }

  // Choose between JointsOscillation() ==normal sinus== or calculateOscillationPACE() ==PACE chirp signal: sinus with frequency increasing
  // :

  // JointsOscillation();
  calculateOscillationPACE();
}

void OscillatorController::applyControlCommand(anymal_motion_control::Command& command) {
  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    const auto nodeEnum = AD::mapActuatorEnumToActuatorNodeEnum::at(actuatorEnum);
    auto& actuatorCommand = command.getActuatorCommands()[actuatorEnum];

    // Set position/velocity control mode for all joints
    actuatorCommand.setMode(series_elastic_actuator::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
    actuatorCommand.setJointPosition(desJointPositions_[actuatorId]);
    // actuatorCommand.setJointVelocity(desJointVelocities_[actuatorId])
    actuatorCommand.setJointVelocity(0);

    // Apply PID gains and zero feedforward torque
    actuatorCommand.setPidGains(params_.gains[nodeEnum]);
    actuatorCommand.setJointTorque(0.0);
  }
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                                OSCILLATION
//==============================================================================

// Normal sinus command with bias, direction and scale but with fixed frequency
void OscillatorController::JointsOscillation() {
  // Fréquence fixe (Hz)
  const double f = 0.1;
  const double phase = 2.0 * PI * f * oscillationTime_;
  const double s = std::sin(phase);

  // === PACE shaping parameters (12 DOF ANYmal) ===
  static constexpr std::array<double, 12> direction = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  static constexpr std::array<double, 12> bias = {0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8};
  static constexpr std::array<double, 12> scale = {0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0};

  for (Eigen::Index i = 0; i < 12; ++i) {
    const double shaped = (s + bias[i]) * direction[i] * scale[i];
    desJointPositions_[i] = shaped;
  }
}

// Chirp sinus command with bias, direction and scale but with frequency increasing in time
// As recommended in the PACE article : frequency from 0.1 Hz to 2 Hz during 20s.
void OscillatorController::calculateOscillationPACE() {
  const double f0 = 0.10;
  const double f1 = 2.00;
  const double T = 20.0;

  const double t = oscillationTime_;
  const double tc = std::min(std::max(t, 0.0), T);

  const double phase = 2.0 * PI * (f0 * tc + ((f1 - f0) / (2.0 * T)) * tc * tc);
  const double chirp = std::sin(phase);

  phaseLast_ = phase;
  chirpLast_ = chirp;
  tcLast_ = tc;
  f0Last_ = f0;
  f1Last_ = f1;
  TLast_ = T;

  static constexpr std::array<double, 12> direction = {1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  static constexpr std::array<double, 12> bias = {0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8, 0.0, 0.4, 0.8};
  static constexpr std::array<double, 12> scale = {0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0, 0.25, 0.5, -2.0};

  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorId = actuatorKey.getId();
    if (actuatorId < 0 || actuatorId >= static_cast<int>(direction.size())) {
      continue;
    }

    const double q = (chirp + bias[actuatorId]) * direction[actuatorId] * scale[actuatorId];
    desJointPositions_[actuatorId] = q;
  }

  phaseLast_ = phase;
  chirpLast_ = chirp;
}
//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                                DEFAULT
//==============================================================================
// This function is triggerd at the begining of the controller Oscillate, to put the robot in a starting pose
void OscillatorController::startGoToDefault(double durationSec) {
  gotoDefaultActive_ = true;
  gotoDefaultTime_ = 0.0;
  gotoDefaultDuration_ = std::max(0.1, durationSec);

  // Pose de départ = là où le robot est maintenant
  gotoDefaultStartPos_ = currentJointPositions_;
}

// This function gives the desJointPositions_ and desJointVelocities_ to bring the robot in default pose linearly
void OscillatorController::GoToDefault(double dt) {
  if (!gotoDefaultActive_) {
    desJointPositions_ = centerPositions_;
    desJointVelocities_.setZero();
    return;
  }

  gotoDefaultTime_ += dt;
  const double T = gotoDefaultDuration_;

  if (gotoDefaultTime_ <= T) {
    const double x = std::clamp(gotoDefaultTime_ / T, 0.0, 1.0);
    const double alpha = x * x * x * (10.0 + x * (-15.0 + 6.0 * x));

    desJointPositions_ = (1.0 - alpha) * gotoDefaultStartPos_ + alpha * gotoDefaultTargetPos_;

    desJointVelocities_ = (gotoDefaultTargetPos_ - gotoDefaultStartPos_) / T;

    return;
  }

  gotoDefaultHoldTime_ += dt;

  desJointPositions_ = gotoDefaultTargetPos_;
  desJointVelocities_.setZero();

  if (gotoDefaultHoldTime_ >= 2.0) {
    gotoDefaultActive_ = false;
    gotoDefaultHoldTime_ = 0.0;
    reset_ = 1.0;
  }
  return;
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                            MOTION CONTROL MANAGER
//==============================================================================
// Switch to a new operation mode (internal implementation)
void OscillatorController::goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) {
  using anymal_motion_control::ReferenceType;
  using anymal_motion_control::SwitchResult;

  // Helper lambda to handle failure cases
  auto fail = [&](const std::string& msg) {
    MELO_WARN_STREAM(msg);
    if (action) action->setAborted();
  };

  // Validate the requested mode
  const auto modeOpt = operationModeFromString(operationMode);
  if (!modeOpt) {
    fail("Unknown operation mode: " + operationMode);
    return;
  }

  // Check compatibility with current reference type
  const auto newMode = *modeOpt;
  const auto currentRefType = activeReferenceType_;

  // TWIST reference type is required for oscillate mode
  const bool isTwistMode = (newMode == OperationMode::OSCILLATE);
  if (isTwistMode && currentRefType != ReferenceType::TWIST) {
    fail("OSCILLATE mode requires TWIST reference type");
    return;
  }

  MELO_INFO_STREAM("Transitioning to " + operationMode + " mode");

  // Handle mode transition
  const auto previousMode = currentMode_;
  currentMode_ = newMode;

  // When switching modes, update center positions to avoid jumps
  if (previousMode != newMode) {
    centerPositions_ = currentJointPositions_;
    oscillationTime_ = 0.0;
    lastUpdateTime_ = Clock::now();
    startGoToDefault(2.0);
  }

  // Report success
  if (action) {
    action->setSucceeded(SwitchResult::SWITCHED);
  }
}

// Switch to a new reference type (internal implementation)
anymal_motion_control::SwitchResult OscillatorController::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
  using anymal_motion_control::ReferenceType;
  using anymal_motion_control::SwitchResult;

  if (referenceType == ReferenceType::TWIST || referenceType == ReferenceType::ACTION) {
    activeReferenceType_ = referenceType;
    return SwitchResult::SWITCHED;
  }

  MELO_WARN_STREAM("Unsupported reference type: " << anymal_motion_control::toString(referenceType));
  return SwitchResult::ERROR;
}

//
//
//
//
//
//
//
//
//
//
//
//
//==============================================================================
//                            AUXILARY METHODS
//==============================================================================
// Prepare controller for stopping
bool OscillatorController::preStop() {
  MELO_INFO_STREAM("Pre-stopping oscillator controller");
  return true;
}
// Stop the controller
bool OscillatorController::stop() {
  MELO_INFO_STREAM("Stopping oscillator controller");

  // Reset controller state
  oscillationTime_ = 0.0;
  twistCommand_.setZero();

  if (logFile_.is_open()) {
    logFile_.flush();
    logFile_.close();
  }

  return true;
}

// Set the twist command input from joystick
void OscillatorController::setTwistCommand(const Twist& twist) {
  twistCommand_ = twist;
}

// Set PID gains for a specific joint type
void OscillatorController::setGain(AD::ActuatorNodeEnum nodeEnum, double pGain, double iGain, double dGain) {
  params_.gains[nodeEnum] = {pGain, iGain, dGain};
}

// Register signals for monitoring and debugging
void OscillatorController::addVariablesToLog() const {
  signal_logger::add(currentJointPositions_, "/joints/current_positions", "controller");
  signal_logger::add(centerPositions_, "/joints/center_positions", "controller/oscillation");
  signal_logger::add(desJointPositions_, "/joints/desired_positions", "controller/oscillation");
  signal_logger::add(desJointVelocities_, "/joints/desired_velocities", "controller/oscillation");
  signal_logger::add(params_.baseAmplitude, "/parameters/base_amplitude", "controller/oscillation");
  signal_logger::add(params_.maxAmplitude, "/parameters/max_amplitude", "controller/oscillation");
  signal_logger::add(params_.frequency, "/parameters/frequency", "controller/oscillation");

  const int modeAsInt = static_cast<int>(currentMode_);
  signal_logger::add(modeAsInt, "/controller/operation_mode", "controller");
  signal_logger::add(twistCommand_, "/twist_command", "controller/input");
}

}  // namespace anymal_ctrl_oscillator
