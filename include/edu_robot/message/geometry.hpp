/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "geometry_msgs/msg/quaternion.hpp"
#include <Eigen/Geometry>

#include <Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace eduart {
namespace robot {
namespace message {

// from ros
inline Eigen::Quaterniond from_ros(const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}
// to ros
inline geometry_msgs::msg::Quaternion to_ros(const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion q_out;

  q_out.w = q.w();
  q_out.x = q.x();
  q_out.y = q.y();
  q_out.z = q.z();

  return q_out;
}

} // end namespace message
} // end namespace robot
} // end namespace eduart
