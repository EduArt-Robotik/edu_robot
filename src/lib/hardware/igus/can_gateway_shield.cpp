#include "edu_robot/hardware/igus/can_gateway_shield.hpp"
#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

CanGatewayShield::CanGatewayShield(char const* const can_device)
  : hardware::can_gateway::CanGatewayShield(can_device)
{

}

CanGatewayShield::CanGatewayShield(char const* const can_device_0, char const* const can_device_1, char const* const can_device_2)
  : hardware::can_gateway::CanGatewayShield(can_device_0, can_device_1, can_device_2)
{

}

CanGatewayShield::~CanGatewayShield()
{

}

void CanGatewayShield::enable()
{
  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->reset();
    motor_controller->enable();
  }
}

void CanGatewayShield::disable()
{
  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->disable();
  }
}

void CanGatewayShield::registerMotorControllerHardware(
  std::shared_ptr<MotorControllerHardware> motor_controller_hardware)
{
  if (std::find(_motor_controller_hardware.begin(), _motor_controller_hardware.end(), motor_controller_hardware)
      != _motor_controller_hardware.end())
  {
    throw std::runtime_error("igus::CanGatewayShield: given motor controller already exists.");
  }

  _motor_controller_hardware.push_back(motor_controller_hardware);
}  

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
