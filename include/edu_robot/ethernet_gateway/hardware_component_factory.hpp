/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"

#include <edu_robot/hardware_component_factory.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetGatewayShield;

class HardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  HardwareComponentFactory(std::shared_ptr<EthernetGatewayShield> shield) : _shield(shield) { }
  ~HardwareComponentFactory() override = default;

  HardwareComponentFactory& addLighting(const std::string& lighting_name, const std::string& hardware_name);
  HardwareComponentFactory& addMotorController(
    const std::string& motor_name, const std::string& hardware_name,
    const eduart::robot::MotorController::Parameter& parameter);
  HardwareComponentFactory& addRangeSensor(const std::string& sensor_name, const std::string& hardware_name,
                                                 const std::uint8_t id, const RangeSensorHardware::Parameter& parameter);
  HardwareComponentFactory& addImuSensor(const std::string& sensor_name, const std::string& hardware_name,
                                               const robot::ImuSensor::Parameter parameter);

private:
  std::shared_ptr<EthernetGatewayShield> _shield;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
