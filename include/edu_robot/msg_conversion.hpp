/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/msg/detail/robot_status_report__struct.hpp"
#include "edu_robot/msg/robot_status_report.hpp"

#include "edu_robot/robot_status_report.hpp"

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

} // end namespace eduart
} // end namespace robot
