/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/action/action.hpp"
#include "edu_robot/motor_controller.hpp"

#include "edu_robot/mode_state_machine.hpp"

namespace eduart {
namespace robot {

class MotorController;

namespace action {

class CheckIfMotorIsEnabled : public FutureAction
{
public:
  CheckIfMotorIsEnabled(
    std::shared_ptr<rclcpp::Clock> clock, const rclcpp::Duration process_in,
    const std::map<std::uint8_t, std::shared_ptr<MotorController>>& motor_controller, StateMachine& state_machine);

  void process() override;

private:
  const std::map<std::uint8_t, std::shared_ptr<MotorController>>& _motor_controller;
  StateMachine& _state_machine;
};

} // end namespace action
} // end namespace eduart
} // end namespace robot
