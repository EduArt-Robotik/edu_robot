/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"

#include <limits>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
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
    _sent_distances[sender] = value;
    float min_front_distance = std::numeric_limits<float>::max();
    float min_rear_distance = std::numeric_limits<float>::max();

    for (const auto& distance : _sent_distances) {
      if (distance.first->name().find("front") != std::string::npos) {
        min_front_distance = std::min(min_front_distance, distance.second);
      }
      else if (distance.first->name().find("rear") != std::string::npos) {
        min_rear_distance = std::min(min_rear_distance, distance.second);
      }
      else {
        throw std::invalid_argument("Collision Avoidance: connected outputs need \"front\" or \"rear\" in their name.");
      }
    }

    _velocity_reduce_factor_front = calculateReduceFactor(min_front_distance, _parameter);
    _velocity_reduce_factor_rear = calculateReduceFactor(min_rear_distance, _parameter);
  }
  // inline float getVelocityReduceFactor() const { return _velocity_reduce_factor; }

private:
  static float calculateReduceFactor(const float input_velocity, const Parameter& parameter)
  {
    if (input_velocity <= parameter.distance_velocity_zero) {
      return 0.0f;
    }
    else if (input_velocity >= parameter.distance_reduce_velocity) {
      return 1.0f;
    }
    // else
    // in distance interval for reducing velocity

    // remove offset (zero velocity distance) from value
    float reduce_factor = (input_velocity - parameter.distance_velocity_zero);
    // normalize distance to zero velocity
    reduce_factor /= (parameter.distance_reduce_velocity - parameter.distance_velocity_zero);

    return reduce_factor;
  }

  const Parameter _parameter;
  float _velocity_reduce_factor_front = 1.0f;
  float _velocity_reduce_factor_rear = 1.0f;
  std::map<const ProcessingComponentOutput<float>*, float> _sent_distances;
};                        

} // end namespace processing
} // end namespace robot
} // end namespace eduart
