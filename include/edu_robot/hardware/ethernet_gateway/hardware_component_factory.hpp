/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include "edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

class EthernetGatewayShield;

class HardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  HardwareComponentFactory(std::shared_ptr<EthernetGatewayShield> shield) : _shield(shield) { }
  ~HardwareComponentFactory() override = default;

  HardwareComponentFactory& addLighting(const std::string& lighting_name);
  HardwareComponentFactory& addMotorController(
    const std::string& controller_name, const MotorControllerHardware<2>::Parameter& parameter);
  HardwareComponentFactory& addSingleChannelMotorController(
    const std::string& controller_name, const MotorControllerHardware<1>::Parameter& parameter);
  HardwareComponentFactory& addRangeSensor(const std::string& sensor_name, const std::uint8_t id);
  HardwareComponentFactory& addImuSensor(const std::string& sensor_name);

private:
  std::shared_ptr<EthernetGatewayShield> _shield;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
