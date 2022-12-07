#pragma once

#include "edu_robot/msg/detail/mode__struct.hpp"
#include "edu_robot/msg/mode.hpp"
#include <cstdint>

namespace eduart {
namespace robot {

enum class Mode {
  UNCONFIGURED              = 0,
  INACTIVE                  = edu_robot::msg::Mode::INACTIVE,
  REMOTE_CONTROLLED         = edu_robot::msg::Mode::REMOTE_CONTROLLED,
  FLEET_MASTER              = edu_robot::msg::Mode::FLEET_MASTER,
  FLEET_SLAVE               = edu_robot::msg::Mode::FLEET_SLAVE,
  MASK_UNSET_DRIVING_MODE   = edu_robot::msg::Mode::MASK_UNSET_DRIVING_MODE,

  SKID_DRIVE                = edu_robot::msg::Mode::SKID_DRIVE,
  MECANUM_DRIVE             = edu_robot::msg::Mode::MECANUM_DRIVE,
  MASK_UNSET_KINEMATIC_MODE = edu_robot::msg::Mode::MASK_UNSET_KINEMATIC_MODE,

  COLLISION_AVOIDANCE_OVERRIDE_ENABLED    = edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_ENABLED,
  COLLISION_AVOIDANCE_OVERRIDE_DISABLED   = edu_robot::msg::Mode::COLLISION_AVOIDANCE_OVERRIDE_DISABLED,
  MASK_UNSET_COLLISION_AVOIDANCE_OVERRIDE = edu_robot::msg::Mode::MASK_UNSET_COLLISION_AVOIDANCE_OVERRIDE,
};

inline Mode& operator|=(Mode& lhs, const Mode rhs)
{
  lhs = static_cast<Mode>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline Mode& operator&=(Mode& lhs, const Mode rhs)
{
  lhs = static_cast<Mode>(static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
  return lhs;
}

inline bool operator&(const Mode lhs, const Mode rhs)
{
  return static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs);
}

} // end namespace robot
} // end namespace eduart
