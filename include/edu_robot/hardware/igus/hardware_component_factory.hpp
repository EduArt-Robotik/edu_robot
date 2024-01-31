/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {

namespace can {
class CanGatewayShield;
} // end namespace can

namespace igus {

class HardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  HardwareComponentFactory(std::shared_ptr<hardware::can::CanGatewayShield> shield) : _shield(shield) { }
  ~HardwareComponentFactory() override = default;

  HardwareComponentFactory& addLighting(const std::string& lighting_name);
  HardwareComponentFactory& addMotorController(const std::string& controller_name, const std::size_t can_id);
  HardwareComponentFactory& addRangeSensor(
    const std::string& sensor_name, const std::uint8_t id, rclcpp::Node& ros_node);
  HardwareComponentFactory& addImuSensor(
    const std::string& sensor_name, rclcpp::Node& ros_node);

private:
  std::shared_ptr<hardware::can::CanGatewayShield> _shield;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
