/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/bot/eduard.hpp"

namespace eduart {
namespace robot {
namespace bot {

class EduardV3 : public robot::bot::Eduard
{
public:
  EduardV3(
    const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns = "");
  ~EduardV3() override;

protected:
  void initialize(eduart::robot::HardwareComponentFactory& factory) override;
};

} // end namespace bot
} // end namespace robot
} // end namespace eduart
