/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/lighting.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/range_sensor.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <map>
#include <memory>
#include <string>

namespace eduart {
namespace robot {

class HardwareComponentFactory
{
public:
  HardwareComponentFactory() = default;
  virtual ~HardwareComponentFactory() = default;

  inline std::map<std::string, std::shared_ptr<Lighting::ComponentInterface>>
  lightingHardware() { return _lighting_hardware; }
  inline std::map<std::string, std::shared_ptr<MotorController::ComponentInterface>>
  motorControllerHardware() { return _motor_controller_hardware; }
  inline std::map<std::string, std::shared_ptr<MotorController::SensorInterface>>
  motorSensorHardware() { return _motor_sensor_hardware; }
  inline std::map<std::string, std::shared_ptr<RangeSensor::SensorInterface>>
  rangeSensorHardware() { return _range_sensor_hardware; }
  inline std::map<std::string, std::shared_ptr<ImuSensor::SensorInterface>>
  imuSensorHardware() { return _imu_sensor_hardware; }

protected:
  std::map<std::string, std::shared_ptr<Lighting::ComponentInterface>> _lighting_hardware;
  std::map<std::string, std::shared_ptr<MotorController::ComponentInterface>> _motor_controller_hardware;
  std::map<std::string, std::shared_ptr<MotorController::SensorInterface>> _motor_sensor_hardware;
  std::map<std::string, std::shared_ptr<RangeSensor::SensorInterface>> _range_sensor_hardware;
  std::map<std::string, std::shared_ptr<ImuSensor::SensorInterface>> _imu_sensor_hardware;
};

} // end namespace robot
} // end namespace eduart
