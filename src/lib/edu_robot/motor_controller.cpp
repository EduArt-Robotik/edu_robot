#include "edu_robot/motor_controller.hpp"

namespace eduart {
namespace robot {

MotorController::MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter)
  : _parameter(parameter)
  , _name(name)
  , _id(id)
{

}

MotorController::~MotorController()
{

}

void MotorController::setRpm(const Rpm rpm)
{
  processSetRpm(rpm);
  _set_rpm = rpm;
}

} // end namespace robot
} // end namespace eduart
