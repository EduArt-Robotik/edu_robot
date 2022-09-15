/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rotation_per_minute.hpp"
#include "edu_robot/lighting.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <vector>

namespace eduart {
namespace robot {

class RobotHardwareInterface
{
public:
  virtual ~RobotHardwareInterface() = default;

  virtual bool enable();
  virtual bool disable();
  virtual RobotStatusReport getStatusReport();
};

} // end namespace eduart
} // end namespace robot