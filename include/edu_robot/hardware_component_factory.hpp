/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/lighting.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/sensor_range.hpp>
#include <edu_robot/sensor_imu.hpp>

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

  inline const std::map<std::string, std::shared_ptr<Lighting::ComponentInterface>>&
  lightingHardware() const { return _lighting_hardware; }
  inline const std::vector<std::shared_ptr<MotorController::HardwareInterface>>&
  motorControllerHardware() const { return _motor_controller_hardware; }
  inline const std::map<std::string, std::shared_ptr<SensorRange::SensorInterface>>&
  rangeSensorHardware() const { return _range_sensor_hardware; }
  inline const std::map<std::string, std::shared_ptr<SensorImu::SensorInterface>>&
  imuSensorHardware() const { return _imu_sensor_hardware; }

protected:
  std::map<std::string, std::shared_ptr<Lighting::ComponentInterface>> _lighting_hardware;
  std::vector<std::shared_ptr<MotorController::HardwareInterface>> _motor_controller_hardware;
  std::map<std::string, std::shared_ptr<SensorRange::SensorInterface>> _range_sensor_hardware;
  std::map<std::string, std::shared_ptr<SensorImu::SensorInterface>> _imu_sensor_hardware;
};

} // end namespace robot
} // end namespace eduart
