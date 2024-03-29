#include "edu_robot/action/motor_action.hpp"
#include "edu_robot/action/action.hpp"

#include "edu_robot/mode.hpp"

#include <edu_robot/motor_controller.hpp>

namespace eduart {
namespace robot {
namespace action {

CheckIfMotorIsEnabled::CheckIfMotorIsEnabled(
  std::shared_ptr<rclcpp::Clock> clock, const rclcpp::Duration process_in,
  const std::map<std::uint8_t, std::shared_ptr<MotorController>>& motor_controller, StateMachine& state_machine)
  : FutureAction(clock, process_in)
  , _motor_controller(motor_controller)
  , _state_machine(state_machine)
{

}

void CheckIfMotorIsEnabled::process()
{
  // Action only processable when robot is in state remote controlled or in state fleet.
  if (_state_machine.mode().robot_mode != RobotMode::REMOTE_CONTROLLED && _state_machine.mode().robot_mode != RobotMode::AUTONOMOUS) {
    // Cancel Processing
    _keep_alive = false;
    return;
  }

  for (const auto& motor : _motor_controller) {
    // If one motor in not enabled than switch to state inactive.
    if (motor.second->isEnabled() == false) {
      motor.second->_lost_enable = true;
      _state_machine.switchToMode(RobotMode::INACTIVE);
      RCLCPP_INFO(rclcpp::get_logger("check_if_motor_enabled"), "switch to mode inactive");
      _keep_alive = false;
      return;
    }
  }

  _keep_alive = true;
}

} // end namespace action
} // end namespace eduart
} // end namespace robot
