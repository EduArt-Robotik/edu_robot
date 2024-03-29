/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/iot_shield/iot_shield.hpp"

#include <edu_robot/hardware_component_factory.hpp>

#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

class IotBotHardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  IotBotHardwareComponentFactory(std::shared_ptr<IotShield> shield) : _shield(shield) { }
  ~IotBotHardwareComponentFactory() override = default;

  IotBotHardwareComponentFactory& addLighting(const std::string& lighting_name);
  IotBotHardwareComponentFactory& addMotorController(const std::string& controller_name);
  IotBotHardwareComponentFactory& addRangeSensor(const std::string& sensor_name, const std::uint8_t id);
  IotBotHardwareComponentFactory& addImuSensor(const std::string& sensor_name);

private:
  std::shared_ptr<IotShield> _shield;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
