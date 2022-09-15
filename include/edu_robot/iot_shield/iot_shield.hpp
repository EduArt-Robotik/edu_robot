/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/robot_hardware_interface.hpp"

namespace eduart {
namespace robot {
namespace iot_bot {

class IotShield : public RobotHardwareInterface
{
public:
  IotShield();
  ~IotShield() override;
  bool enable() override;
  bool disable() override;
  RobotStatusReport getStatusReport() override;
};

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
