#include "edu_robot/processing_component/collison_avoidance.hpp"

namespace eduart {
namespace robot {
namespace processing {

CollisionAvoidance::CollisionAvoidance(const Parameter parameter, rclcpp::Node& ros_node)
  : ProcessingComponent("collision avoidance", ros_node)
  , _parameter(parameter)
{
  createInput<float>("range.front.right", 10);
  createInput<float>("range.front.left", 10);
  createInput<float>("range.rear.right", 10);
  createInput<float>("range.rear.left", 10);
}

void CollisionAvoidance::process()
{
  // first empty queue and only get last value, because only last one counts
  // if there is no new value present than the last received will be used to compute the reduce factor
  for (auto port = input("range.front.right"); port->hasValue();) {
    _last_distance_front_right = port->getValue().get<float>();
  }
  for (auto port = input("range.front.left"); port->hasValue();) {
    _last_distance_front_left = port->getValue().get<float>();
  }  
  for (auto port = input("range.rear.right"); port->hasValue();) {
    _last_distance_rear_right = port->getValue().get<float>();
  }  
  for (auto port = input("range.rear.left"); port->hasValue();) {
    _last_distance_rear_left = port->getValue().get<float>();
  }  

  // estimate the two closest distances
  float min_front_distance = std::min(_last_distance_front_left, _last_distance_front_right);
  float min_rear_distance  = std::min(_last_distance_rear_left, _last_distance_rear_right);

  _velocity_reduce_factor_front = calculateReduceFactor(min_front_distance, _parameter);
  _velocity_reduce_factor_rear  = calculateReduceFactor(min_rear_distance, _parameter);
}

float CollisionAvoidance::calculateReduceFactor(const float input_velocity, const Parameter& parameter)
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

} // end namespace processing
} // end namespace robot
} // end namespace eduart
