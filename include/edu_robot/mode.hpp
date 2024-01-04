#pragma once

#include "edu_robot/msg/mode.hpp"

#include <cstdint>

namespace eduart {
namespace robot {

enum class RobotMode {
  UNCONFIGURED              = 0,
  INACTIVE                  = edu_robot::msg::Mode::INACTIVE,
  REMOTE_CONTROLLED         = edu_robot::msg::Mode::REMOTE_CONTROLLED,
  AUTONOMOUS                = edu_robot::msg::Mode::AUTONOMOUS,
  CHARGING                  = edu_robot::msg::Mode::CHARGING,
  // MASK_UNSET_DRIVING_MODE   = edu_robot::msg::Mode::MASK_UNSET_DRIVING_MODE,
};

enum class DriveKinematic {
  SKID_DRIVE                = edu_robot::msg::Mode::SKID_DRIVE,
  MECANUM_DRIVE             = edu_robot::msg::Mode::MECANUM_DRIVE,
  // MASK_UNSET_KINEMATIC_MODE = edu_robot::msg::Mode::MASK_UNSET_KINEMATIC_MODE,
};

enum class FeatureMode {
  NONE = 0,
  COLLISION_AVOIDANCE_OVERRIDE = edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE,
  COLLISION_AVOIDANCE          = edu_robot::msg::Mode::COLLISION_AVOIDANCE,
};

struct Mode {
  // Robot mode. Only one mode can be active.
  RobotMode robot_mode = RobotMode::UNCONFIGURED; 
  // The drive kinematic used by the robot for driving. Only one kinematic can be active.
  DriveKinematic drive_kinematic = DriveKinematic::SKID_DRIVE; 
  // Represents robot features like collision avoidance. The features can be combined.
  FeatureMode feature_mode = FeatureMode::NONE; 
};

inline FeatureMode& operator|=(FeatureMode& lhs, const FeatureMode rhs)
{
  lhs = static_cast<FeatureMode>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline FeatureMode& operator&=(FeatureMode& lhs, const FeatureMode rhs)
{
  lhs = static_cast<FeatureMode>(static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline bool operator&(const FeatureMode lhs, const FeatureMode rhs)
{
  return static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs);
}

inline edu_robot::msg::Mode to_ros(const Mode& mode)
{
  edu_robot::msg::Mode msg;

  msg.mode = static_cast<std::uint8_t>(mode.robot_mode);
  msg.drive_kinematic = static_cast<std::uint8_t>(mode.drive_kinematic);
  msg.feature_mode = static_cast<std::uint8_t>(mode.feature_mode);

  return msg;
}

} // end namespace robot
} // end namespace eduart
