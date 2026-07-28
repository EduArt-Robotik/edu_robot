#include "edu_robot/action/indicate_rejected_action.hpp"

namespace eduart {
namespace robot {
namespace action {

IndicateRejected::IndicateRejected(
  std::shared_ptr<rclcpp::Clock> clock, rclcpp::Duration process_interval,
  std::function<void(const RobotMode)> set_lighting, StateMachine& state_machine)
  : _clock(clock),
    _process_until(clock->now() + process_interval),
    _set_lighting(set_lighting),
    _state_machine(state_machine)
{

}

bool IndicateRejected::isReady() const
{
  return true;
}

void IndicateRejected::process()
{
  if (_clock->now() >= _process_until)
  {
    // set lighting back for specific mode
    _set_lighting(_state_machine.mode().robot_mode);
    _keep_alive = false;
    return;
  }
  if (_set_lighting_once == false) {
    _set_lighting(RobotMode::BATTERY_EMPTY);
    _set_lighting_once = true;
  }

  _keep_alive = true;
}

} // end namespace action
} // end namespace eduart
} // end namespace robot

