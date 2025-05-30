/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/plane.hpp>

namespace eduart {
namespace robot {
namespace algorithm {

template <typename DataType>
bool is_above_plane(const Plane<DataType>& plane, const Eigen::Vector<DataType, 3>& point) {
  return plane.n().dot(point - plane.origin()) > 0;
}

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
