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

protected:
  std::map<std::string, std::shared_ptr<HardwareComponentInterface<Color, Lighting::Mode>>> _lighting_hardware;
  std::map<std::string, std::shared_ptr<HardwareComponentInterface<Rpm>>> _motor_controller_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Rpm>>> _motor_sensor_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<float>>> _range_sensor_hardware;
  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Eigen::Quaterniond>>> _imu_sensor_hardware;
};

} // end namespace robot
} // end namespace eduart
