/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include "edu_robot/hardware/can/sensor_point_cloud_hardware.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

class CanGatewayShield;

class HardwareComponentFactory : public eduart::robot::HardwareComponentFactory
{
public:
  HardwareComponentFactory(std::shared_ptr<CanGatewayShield> shield) : _shield(shield) { }
  ~HardwareComponentFactory() override = default;

  HardwareComponentFactory& addLighting(const std::string& lighting_name);
  HardwareComponentFactory& addMotorController(const std::string& controller_name, const std::size_t can_id);
  HardwareComponentFactory& addRangeSensor(
    const std::string& sensor_name, const std::uint8_t id, rclcpp::Node& ros_node);
  HardwareComponentFactory& addImuSensor(const std::string& sensor_name, rclcpp::Node& ros_node);
  HardwareComponentFactory& addPointCloudSensor(
    const std::string& sensor_name, const SensorPointCloudHardware::Parameter& parameter, rclcpp::Node& ros_node);

protected:
  std::shared_ptr<CanGatewayShield> _shield;
};

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
