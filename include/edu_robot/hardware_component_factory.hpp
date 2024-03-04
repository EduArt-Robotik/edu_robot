/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_interface.hpp>
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

  inline const std::vector<std::shared_ptr<MotorController::HardwareInterface>>&
  motorControllerHardware() const { return _motor_controller_hardware; }
  inline const std::map<std::string, std::shared_ptr<HardwareInterface>>& hardware() const {
    return _hardware;
  }

protected:
  std::vector<std::shared_ptr<MotorController::HardwareInterface>> _motor_controller_hardware;
  std::map<std::string, std::shared_ptr<HardwareInterface>> _hardware;
};

} // end namespace robot
} // end namespace eduart
