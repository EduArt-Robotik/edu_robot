/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <memory>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace eduart {
namespace robot {
namespace action {

class Action
{
public:
  virtual ~Action() = default;

  virtual bool isReady() const = 0;
  virtual void process() = 0;
};

class FutureAction : public Action
{
public:
  FutureAction(std::shared_ptr<rclcpp::Clock> clock, rclcpp::Duration process_in)
    : _clock(clock)
    , _process_at(clock->now() + process_in)  
  { }

  bool isReady() const override {
    return (_clock->now() > _process_at);
  }

private:
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _process_at;
};

} // end namespace action
} // end namespace eduart
} // end namespace robot
