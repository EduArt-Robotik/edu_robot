/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class CanGatewayShield;

class HardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  HardwareComponentFactory(std::shared_ptr<CanGatewayShield> shield) : _shield(shield) { }
  ~HardwareComponentFactory() override = default;

  HardwareComponentFactory& addLighting(const std::string& lighting_name);
  HardwareComponentFactory& addMotorController(
    const std::string& controller_name, const std::uint32_t can_id_input, const std::uint32_t can_id_output);
  HardwareComponentFactory& addRangeSensor(
    const std::string& sensor_name, const std::uint8_t id);
  HardwareComponentFactory& addImuSensor(
    const std::string& sensor_name, const std::uint32_t can_id);
  HardwareComponentFactory& addTofSensor(
    const std::string& sensor_name, const SensorTofHardware::Parameter& parameter, rclcpp::Node& ros_node);
  HardwareComponentFactory& addTofRingSensor(
    const std::string& sensor_name, const SensorTofRingHardware::Parameter& parameter, rclcpp::Node& ros_node);

protected:
  std::shared_ptr<CanGatewayShield> _shield;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
