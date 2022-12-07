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
    float distance_reduce_velocity = 0.4f;
    float distance_velocity_zero = 0.05;
  };

  CollisionAvoidance(const Parameter parameter, rclcpp::Node& ros_node);
  ~CollisionAvoidance() override = default;

  void processInput(const float& value, const ProcessingComponentOutput<float>* sender) override;
  inline float getVelocityReduceFactorFront() const { return _velocity_reduce_factor_front; }
  inline float getVelocityReduceFactorRear() const { return _velocity_reduce_factor_rear; }

private:
  static float calculateReduceFactor(const float input_velocity, const Parameter& parameter);

  const Parameter _parameter;
  float _velocity_reduce_factor_front = 1.0f;
  float _velocity_reduce_factor_rear = 1.0f;
  std::map<const ProcessingComponentOutput<float>*, float> _sent_distances;
};                        

} // end namespace processing
} // end namespace robot
} // end namespace eduart
