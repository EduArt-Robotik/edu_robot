/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/angle.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Geometry>

namespace eduart {
namespace robot {
namespace algorithm {

AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
AnglePiToPi quaternion_to_yaw(const Eigen::Quaterniond& q);

void eliminate_yaw(geometry_msgs::msg::Quaternion& q);
void eliminate_yaw(Eigen::Quaterniond& q);

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
