/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/mode.hpp"

#include <map>

namespace eduart {
namespace robot {
namespace mode {

// Defines if a mode transition is possible. By default no transition is allowed. By specializing the template
// transitions can be enabled.
template <RobotMode From, RobotMode To> struct can_switch_to_mode { static constexpr bool value = false; };

template <> struct can_switch_to_mode<RobotMode::UNCONFIGURED, RobotMode::INACTIVE>      { static constexpr bool value = true; };

template <> struct can_switch_to_mode<RobotMode::INACTIVE, RobotMode::REMOTE_CONTROLLED> { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::INACTIVE, RobotMode::FLEET>             { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::INACTIVE, RobotMode::CHARGING>          { static constexpr bool value = true; };

template <> struct can_switch_to_mode<RobotMode::REMOTE_CONTROLLED, RobotMode::INACTIVE> { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::REMOTE_CONTROLLED, RobotMode::FLEET>    { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::REMOTE_CONTROLLED, RobotMode::CHARGING> { static constexpr bool value = true; };

template <> struct can_switch_to_mode<RobotMode::FLEET, RobotMode::INACTIVE>             { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::FLEET, RobotMode::REMOTE_CONTROLLED>    { static constexpr bool value = true; };
template <> struct can_switch_to_mode<RobotMode::FLEET, RobotMode::CHARGING>             { static constexpr bool value = true; };

template <> struct can_switch_to_mode<RobotMode::CHARGING, RobotMode::INACTIVE>          { static constexpr bool value = true; };

// Defines the operation that should be performed on a mode change. By default nothing will be done. Just the new mode will be assigned.
template <RobotMode From, RobotMode To> struct switch_mode {
  inline static void perform(Mode& mode) { mode.robot_mode = To; }
};

template <RobotMode From> struct switch_mode<From, RobotMode::FLEET> {
  inline static void perform(Mode& mode) {
    mode.drive_kinematic = DriveKinematic::MECANUM_DRIVE;
    mode.feature_mode |= FeatureMode::COLLISION_AVOIDANCE_OVERRIDE;
    mode.robot_mode = RobotMode::FLEET;
  }
};

} // end namespace mode

/**
 * \brief Mode state machine constructs a mode mapping from each mode to each. Each transition of the modes need to be
 *        defined by specializing the template classes and methods in namespace mode.
 */
template <RobotMode... Modes>
class ModeStateMachine
{
  static constexpr std::size_t num_modes = sizeof...(Modes);

public:
  using ActivationOperation   = std::function<void()>;
  using DeactivationOperation = std::function<void()>;

  void setModeActivationOperation(const RobotMode mode, const ActivationOperation& operation) {
    _activation_operation[mode] = operation;
  }
  void setModeDeactivationOperation(const RobotMode mode, const DeactivationOperation& operation) {
    _deactivation_operation[mode] = operation;
  }

  void switchToMode(const RobotMode mode) {
    // Performing Mode Switch
    const auto current_robot_mode = _current_mode.robot_mode;
    if (((_current_mode.robot_mode == Modes ? performSwitchingMode<Modes>(_current_mode) : false) || ...) == false) {
      throw std::runtime_error("Mode State Machine: can't do transition.");
    }

    // Calling Deactivation Operation if exists.
    const auto search_deactivation = _deactivation_operation.find(current_robot_mode);
    search_deactivation != _deactivation_operation.end() ? search_deactivation->second() : (void)mode;

    // Calling Activation Operation if exists.
    const auto search_activation = _activation_operation.find(mode);
    search_activation != _activation_operation.end() ? search_activation->second() : (void)mode;
  }
  inline Mode mode() const { return _current_mode; }
  inline void enableFeature(const FeatureMode feature) {
    _current_mode.feature_mode |= feature;
  }
  inline void disableFeature(const FeatureMode feature) {
    _current_mode.feature_mode &= static_cast<FeatureMode>(~static_cast<std::uint8_t>(feature));
  }
  inline void setDriveKinematic(const DriveKinematic kinematic) {
    _current_mode.drive_kinematic = kinematic;
  }

private:
  template <RobotMode From>
  inline bool performSwitchingMode(Mode& mode) const {
    return (((mode.robot_mode == Modes) && mode::can_switch_to_mode<From, Modes>::value ? mode::switch_mode<From, Modes>::perform(mode), true : false) || ...);
  }

  Mode _current_mode;
  std::map<RobotMode, ActivationOperation> _activation_operation;
  std::map<RobotMode, DeactivationOperation> _deactivation_operation;
};

} // end namespace robot
} // end namespace eduart
