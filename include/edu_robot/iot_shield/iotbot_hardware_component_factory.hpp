/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"

#include <edu_robot/range_sensor.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <map>
#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

// NOTE: it is just a try to use a factory for creating and configuring hardware components for IoTBot.
class IotBotHardwareComponentFactory
{
public:
  IotBotHardwareComponentFactory(std::shared_ptr<IotShield> shield) : _shield(shield) { }

  IotBotHardwareComponentFactory& addLighting(const std::string& lighting_name, const std::string& hardware_name);
  IotBotHardwareComponentFactory& addMotorController(const std::string& motor_name, const std::string& hardware_name,
                                                     const eduart::robot::MotorController::Parameter& parameter);
  IotBotHardwareComponentFactory& addRangeSensor(const std::string& sensor_name, const std::string& hardware_name,
                                                 const std::uint8_t id, const robot::RangeSensor::Parameter& parameter);
  IotBotHardwareComponentFactory& addImuSensor(const std::string& sensor_name, const std::string& hardware_name,
                                               const robot::ImuSensor::Parameter parameter);

  inline std::map<std::string, std::shared_ptr<HardwareComponentInterface<Color, Lighting::Mode>>>
  lightingHardware() { return _lighting_hardware; }
  inline std::map<std::string, std::shared_ptr<HardwareComponentInterface<Rpm>>>
  motorControllerHardware() { return _motor_controller_hardware; }
  inline std::map<std::string, std::shared_ptr<HardwareSensorInterface<Rpm>>>
  motorSensorHardware() { return _motor_sensor_hardware; }
  inline std::map<std::string, std::shared_ptr<HardwareSensorInterface<float>>>
  rangeSensorHardware() { return _range_sensor_hardware; }
  inline std::map<std::string, std::shared_ptr<HardwareSensorInterface<Eigen::Quaterniond>>>
  imuSensorHardware() { return _imu_sensor_hardware; }

private:
  std::shared_ptr<IotShield> _shield;

  std::map<std::string, std::shared_ptr<HardwareComponentInterface<Color, Lighting::Mode>>> _lighting_hardware;
  std::map<std::string, std::shared_ptr<HardwareComponentInterface<Rpm>>> _motor_controller_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Rpm>>> _motor_sensor_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<float>>> _range_sensor_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Eigen::Quaterniond>>> _imu_sensor_hardware;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
