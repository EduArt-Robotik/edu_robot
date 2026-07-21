/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/algorithm/controller_interface.hpp"

namespace eduart {
namespace robot {
namespace algorithm {

class Pid : public ControllerInterface
{
public:
  struct Parameter {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double limit = 10.0;
    double input_filter_weight = 1.0;
    bool use_anti_windup = true;
  } parameter;

  Pid() = default;
  Pid(const Parameter& parameter_) : parameter(parameter_) { }
  ~Pid() override = default;
  void reset() override;
  double process(const double set_point, const double feedback, const double dt) override;
  double operator()(const double set_point, const double feedback, const double dt) {
    return process(set_point, feedback, dt);
  }

private:
  double _e_integral = 0.0;
  double _e_prev = 0.0;
  double _set_point_prev = 0.0;
  double _previous_feedback = 0.0;
};

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
