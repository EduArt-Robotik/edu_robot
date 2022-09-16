/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/msg/detail/set_lighting_color__struct.hpp"
#include "edu_robot/msg/robot_status_report.hpp"
#include "edu_robot/msg/set_lighting_color.hpp"

#include "edu_robot/robot_status_report.hpp"
#include "edu_robot/lighting.hpp"
#include <stdexcept>

namespace eduart {
namespace robot {

inline edu_robot::msg::RobotStatusReport toRos(const RobotStatusReport& report)
{
  edu_robot::msg::RobotStatusReport msg;

  msg.temperature   = report.temperature;
  msg.mcu_voltage   = report.voltage.mcu;
  msg.mcu_current   = report.current.mcu;
  msg.mcu_current   = report.current.mcu;
  msg.drive_voltage = report.current.drive;

  return msg;
}

inline Lighting::Mode fromRos(const edu_robot::msg::SetLightingColor& msg)
{
  switch (msg.mode) {
  case 0u: // OFF
    return Lighting::Mode::OFF;
  case 1u: // DIM
    return Lighting::Mode::DIM;
  case 2u: // FLASH
    return Lighting::Mode::FLASH;
  case 3u: // PULSATION
    return Lighting::Mode::PULSATION;
  case 4u: // ROTATION
    return Lighting::Mode::ROTATION;
  case 5u: // RUNNING
    return Lighting::Mode::RUNNING;
  default:
    throw std::invalid_argument("Unsupported Lighting::Mode entry.");
  }
}

} // end namespace eduart
} // end namespace robot
