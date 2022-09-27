/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rotation_per_minute.hpp"

#include <limits>
#include <vector>

namespace eduart {
namespace robot {

struct RobotStatusReport
{
  float temperature = std::numeric_limits<float>::quiet_NaN();

  struct {
    float mcu   = std::numeric_limits<float>::quiet_NaN();
    float drive = std::numeric_limits<float>::quiet_NaN();
  } voltage;

  struct {
    float mcu   = std::numeric_limits<float>::quiet_NaN();
    float drive = std::numeric_limits<float>::quiet_NaN();
  } current;

  std::vector<Rpm> rpm;
};

} // end namespace eduart
} // end namespace robot
