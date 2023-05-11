/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>

namespace eduart {
namespace robot {

/**
 * \brief Base class of each sensor. Provides general sensor configuration for example used for tf publishing.
 */
class Sensor
{
protected:
  Sensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
         const tf2::Transform sensor_transform);

public:
  Sensor(const Sensor&) = delete;
  virtual ~Sensor() = default;

  inline const std::string& name() const { return _name; }
  inline const std::string& frameId() const { return _frame_id; }
  
  geometry_msgs::msg::TransformStamped getTransformMsg(const rclcpp::Time stamp) const;

private:
  std::string _name;
  std::string _frame_id;
  std::string _reference_frame_id;
  tf2::Transform _sensor_transform;
};

} // end namespace eduart
} // end namespace robot
