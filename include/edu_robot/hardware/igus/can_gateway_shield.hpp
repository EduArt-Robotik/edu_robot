/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/can/can_gateway_shield.hpp>

#include <edu_robot/hardware/communicator.hpp>

#include <edu_robot/hardware_robot_interface.hpp>
#include <edu_robot/robot_status_report.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

class MotorControllerHardware;

class CanGatewayShield : public hardware::can::CanGatewayShield
{
public:
  CanGatewayShield(char const* const can_device);
  CanGatewayShield(char const* const can_device_0, char const* const can_device_1, char const* const can_device_2);  
  ~CanGatewayShield() override;
  void enable() override;
  void disable() override;
  void registerMotorControllerHardware(std::shared_ptr<MotorControllerHardware> motor_controller_hardware);

private:
  std::vector<std::shared_ptr<MotorControllerHardware>> _motor_controller_hardware;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
