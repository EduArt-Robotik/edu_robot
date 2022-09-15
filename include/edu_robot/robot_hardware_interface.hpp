/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rotation_per_minute.hpp"

#include <vector>

namespace eduart {
namespace robot {

class RobotHardwareInterface
{
public:
  virtual ~RobotHardwareInterface() = default;

  virtual bool enable();
  virtual bool disable();
  virtual bool setRpm(const std::vector<Rpm>& rpm);
  virtual const std::vector<Rpm>& getRpm() const;
};

} // end namespace eduart
} // end namespace robot