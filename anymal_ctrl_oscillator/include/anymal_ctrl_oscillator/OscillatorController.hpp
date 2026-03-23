#pragma once

// std
#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

// ANYmal framework
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_model/AnymalState.hpp>
#include <anymal_motion_control/AnymalController.hpp>
#include <series_elastic_actuator/SeActuatorCommand.hpp>

// Eigen
#include <Eigen/Core>

#include <fstream>
#include <string>

namespace anymal_ctrl_oscillator {

class OscillatorController : public anymal_motion_control::AnymalController {
 public:
  void startGoToDefault(double durationSec);
  void GoToDefault(double dt);

  using AD = anymal_description::AnymalDescription;

  using JointPositions = Eigen::Matrix<double, AD::getJointsDimension(), 1>;
  using JointVelocities = Eigen::Matrix<double, AD::getJointsDimension(), 1>;
  using Twist = anymal_model::Twist;
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::duration<double>;
  using TimePoint = std::chrono::time_point<Clock, Duration>;

  // ==============================================
  // NEW OPERATION MODES
  // ==============================================
  enum class OperationMode { OSCILLATE, DEFAULT };

  struct Parameters {
    std::unordered_map<AD::ActuatorNodeEnum, series_elastic_actuator::SeActuatorCommand::PidGains> gains{
        {AD::ActuatorNodeEnum::HAA, {250.0, 0.2, 2.0}},
        {AD::ActuatorNodeEnum::HFE, {200.0, 0.1, 1.5}},
        {AD::ActuatorNodeEnum::KFE, {150.0, 0.05, 1.0}}};

    double baseAmplitude = 0.02;
    double maxAmplitude = 0.10;
    double frequency = 0.5;
    double minimumVelocityThreshold = 0.001;

    double commandScale = 1.0;

    double rampUpDuration = 1.0;
    double stabilizationPeriod = 0.0;
  };

  OscillatorController() = default;

  bool create() override;
  bool doInitialize(const anymal_model::AnymalState state) override;
  bool advance(const anymal_model::AnymalState state, anymal_motion_control::Command& command) override;

  bool preStop() override;
  bool stop() override;

  void setTwistCommand(const Twist& twist);
  void setGain(AD::ActuatorNodeEnum nodeEnum, double pGain, double iGain, double dGain);

  void resetCenterPositions();
  void setParams(Parameters params) { params_ = std::move(params); }
  [[nodiscard]] const Parameters& getParams() const& noexcept { return params_; }

  void addVariablesToLog() const;

 private:
  double chirpLast_ = 0.0;
  double phaseLast_ = 0.0;
  double tcLast_ = 0.0;
  double f0Last_ = 0.0;
  double f1Last_ = 0.0;
  double TLast_ = 0.0;
  std::ofstream logFile_;
  bool logEnabled_ = true;
  std::size_t logEveryN_ = 10;  // log 1 sample every N advances (ex: if 400 Hz -> N=10 => 40 Hz)
  std::size_t logCounter_ = 0;
  std::string logPath_ = "/tmp/anymal_pace_log.csv";
  double gotoDefaultHoldTime_{0.0};

  void setJointsCommand(double dt);

  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;

  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

  void applyControlCommand(anymal_motion_control::Command& command);
  void JointsOscillation();
  void calculateOscillationPACE();

  [[nodiscard]] double calculateAmplitudeFromVelocity(double forwardVelocity, double elapsedTime) const;

  void updateCenterPositionsFromState(const anymal_model::AnymalState& state);

  // ==============================================
  // NEW OPERATION MODE MAP
  // ==============================================
  static inline constexpr std::array<std::pair<std::string_view, OperationMode>, 2> operationModeMap{
      {{"oscillate", OperationMode::OSCILLATE}}};

  [[nodiscard]] static std::optional<OperationMode> operationModeFromString(std::string_view name) {
    for (const auto& [modeName, modeValue] : operationModeMap) {
      if (modeName == name) return modeValue;
    }
    return std::nullopt;
  }

  Parameters params_{};

  JointPositions currentJointPositions_{JointPositions::Zero()};
  JointPositions centerPositions_{JointPositions::Zero()};
  JointPositions desJointPositions_{JointPositions::Zero()};
  JointVelocities desJointVelocities_{JointVelocities::Zero()};

  double oscillationTime_{0.0};
  TimePoint lastUpdateTime_{Clock::now()};
  double reset_{0.0};

  Twist twistCommand_{};

  // ==============================================
  // DEFAULT MODE AT START
  // ==============================================
  OperationMode currentMode_{OperationMode::OSCILLATE};

  anymal_motion_control::ReferenceType activeReferenceType_{anymal_motion_control::ReferenceType::NA};

  bool gotoDefaultActive_ = false;
  double gotoDefaultTime_ = 0.0;
  double gotoDefaultDuration_ = 2.0;
  double first_time = 1.0;

  Eigen::VectorXd gotoDefaultStartPos_;
  Eigen::VectorXd gotoDefaultTargetPos_;
};

}  // namespace anymal_ctrl_oscillator
