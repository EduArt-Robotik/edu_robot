/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/mode.hpp>
#include <edu_robot/mode_state_machine.hpp>

#include "edu_robot/action/action.hpp"

namespace eduart {
namespace robot {

class Lighting;

namespace action {

class IndicateRejected : public Action
{
public:
  IndicateRejected(
    std::shared_ptr<rclcpp::Clock> clock, rclcpp::Duration process_interval,
    std::function<void(const RobotMode)> set_lighting, StateMachine& state_machine);
  ~IndicateRejected() override = default;

  bool isReady() const override;
  void process() override;

private:
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _process_until;
  std::function<void(const RobotMode)> _set_lighting;
  StateMachine& _state_machine;
  bool _set_lighting_once = false;
};

} // end namespace action
} // end namespace eduart
} // end namespace robot