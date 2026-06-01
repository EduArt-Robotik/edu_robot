/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_factory.hpp>

#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"

#include <cstdint>
#include <memory>
#include <string>

// Forward declaration for sensorring type
namespace eduart {
namespace sensorring {
namespace manager { class MeasurementManager; }
} // end namespace sensorring
} // end namespace eduart

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

  HardwareComponentFactory& addLighting();
  HardwareComponentFactory& addMotorController(
    const std::string& controller_name, const MotorControllerHardware::Parameter& parameter);
  //HardwareComponentFactory& addRangeSensor(
  //  const std::string& sensor_name, const std::uint8_t id);
  HardwareComponentFactory& addImuSensor(
    const std::string& sensor_name, const std::uint32_t can_id);
  HardwareComponentFactory& addTofSensor(
    const std::string& sensor_name, const SensorTofHardware::Parameter& parameter, rclcpp::Node& ros_node);

  /**
   * \brief Add a sensor ring (left + right) backed by edu_lib_sensorring when available,
   *        falling back to the legacy SensorTofRingHardware otherwise.
   *
   * \note addSensorRing() must be called before addLighting() so that _measurement_manager
   *       is populated when the lighting manager is initialized.
   */
  HardwareComponentFactory& addSensorRing(
    const std::string& sensor_name, const std::vector<std::string>& left_sensor_names,
    const std::vector<std::string>& right_sensor_names, rclcpp::Node& ros_node);

protected:
  std::shared_ptr<CanGatewayShield> _shield;

  // Populated by addSensorRing(); consumed by addLighting().
  std::shared_ptr<sensorring::manager::MeasurementManager> _measurement_manager;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
