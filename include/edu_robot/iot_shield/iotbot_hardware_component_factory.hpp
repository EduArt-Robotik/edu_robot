/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/range_sensor_hardware.hpp"

#include <edu_robot/eduard/eduard_hardware_component_factory.hpp>

#include <map>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

class IotBotHardwareComponentFactory : public eduard::EduardHardwareComponentFactory
{
public:
  IotBotHardwareComponentFactory(std::shared_ptr<IotShield> shield) : _shield(shield) { }
  ~IotBotHardwareComponentFactory() override = default;

  IotBotHardwareComponentFactory& addLighting(const std::string& lighting_name, const std::string& hardware_name);
  IotBotHardwareComponentFactory& addMotorController(const std::string& motor_name, const std::string& hardware_name,
                                                     const eduart::robot::MotorController::Parameter& parameter);
  IotBotHardwareComponentFactory& addRangeSensor(const std::string& sensor_name, const std::string& hardware_name,
                                                 const std::uint8_t id, const RangeSensorHardware::Parameter& parameter);
  IotBotHardwareComponentFactory& addImuSensor(const std::string& sensor_name, const std::string& hardware_name,
                                               const robot::ImuSensor::Parameter parameter);

private:
  std::shared_ptr<IotShield> _shield;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
