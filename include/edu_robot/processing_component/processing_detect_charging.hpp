/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"

#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace processing {

class DetectCharging : public ProcessingComponent
                     , public ProcessingComponentInput<float>
{
public:
  struct Parameter {
    float voltage_threshold = 28.5f;
  };

  DetectCharging(const Parameter parameter, rclcpp::Node& ros_node)
    : ProcessingComponent("detect_charging", ros_node)
    , _parameter(parameter)
  { }
  ~DetectCharging() override = default;

  void processInput(const float& value, const ProcessingComponentOutput<float>*) override
  {
    _is_charging = value >= _parameter.voltage_threshold;
  }

  inline bool isCharging() const { return _is_charging; }

private:
  Parameter _parameter;
  bool _is_charging = false;
};                    

} // end namespace processing
} // end namespace robot
} // end namespace eduart
