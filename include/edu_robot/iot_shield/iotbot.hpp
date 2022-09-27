/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/robot.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class IotBot : public eduart::robot::Robot
{
public:
  IotBot();
  ~IotBot() override;

  void initialize();
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
