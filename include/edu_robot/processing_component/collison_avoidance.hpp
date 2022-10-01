/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"
#include <rclcpp/node.hpp>
#include <vector>

namespace eduart {
namespace robot {
namespace processing {

class CollisionAvoidance : public ProcessingComponent
                         , public ProcessingComponentInput<float>
{
public:
  struct Parameter {
    float distance_reduce_velocity = 0.3f;
    float distance_velocity_zero = 0.05;
  };

  CollisionAvoidance(const Parameter parameter, rclcpp::Node& ros_node)
    : ProcessingComponent("collision avoidance", ros_node)
    , _parameter(parameter)
  { }
  ~CollisionAvoidance() override = default;

  void processInput(const float& value, const ProcessingComponentOutput<float>* sender) override
  {
    if (value <= _parameter.distance_velocity_zero) {
      _velocity_reduce_factor = 0.0f;
      _sender_sent[sender] = false;
    }
    else if (value <= _parameter.distance_reduce_velocity) {
      // remove offset (zero velocity distance) from value
      float reduce_factor = (value - _parameter.distance_velocity_zero);
      // normalize distance to zero velocity
      reduce_factor /= (_parameter.distance_reduce_velocity - _parameter.distance_velocity_zero);
      // only take new value when factor is lower then previous one
      _velocity_reduce_factor = std::min(reduce_factor, _velocity_reduce_factor);
      _sender_sent[sender] = false;
    }
    else {
      _sender_sent[sender] = true;

      for (const auto& sender : _sender_sent) {
        // if one hasn't sent --> cancel releasing velocity factor
        if (sender.second == false) {
          return;
        }
      }

      // all send a distance greater than distance_reduce_velocity --> remove reduction
      _velocity_reduce_factor = 1.0f;
    }
  }
  inline float getVelocityReduceFactor() const { return _velocity_reduce_factor; }

private:
  const Parameter _parameter;
  float _velocity_reduce_factor = 1.0f;
  std::map<const ProcessingComponentOutput<float>*, bool> _sender_sent;
};                        

} // end namespace processing
} // end namespace robot
} // end namespace eduart
