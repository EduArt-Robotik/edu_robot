/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/eduard/eduard.hpp>

namespace eduart {
namespace robot {
namespace iotbot {

class IotBot : public eduart::robot::eduard::Eduard
{
public:
  IotBot();
  ~IotBot() override;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
