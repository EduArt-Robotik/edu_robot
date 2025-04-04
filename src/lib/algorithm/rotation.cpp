#include "edu_robot/algorithm/rotation.hpp"
#include "edu_robot/angle.hpp"
#include "edu_robot/msg_conversion.hpp"

#include <edu_robot/message/geometry.hpp>

#include <Eigen/Geometry>

#include <iostream>

namespace eduart {
namespace robot {
namespace algorithm {

AnglePiToPi quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  // return std::atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
  const auto q_in = message::from_ros(q);
  const Eigen::Vector3d b_x = Eigen::Vector3d::UnitX();

  std::cout << "transformed b_x:" << b_x.transpose() << std::endl;
  const Eigen::Vector3d b_x_transformed = q_in * b_x;
  const Eigen::Vector2d b_x_transformed_2d = Eigen::Vector2d(b_x_transformed.x(), b_x_transformed.y()).normalized();

  const double dot = Eigen::Vector2d::UnitX().dot(b_x_transformed_2d);
  const double det = b_x_transformed_2d.y(); // det = x1 * y2 - y1 * x2 = 1 * y2 - 0 * x2 = y2
  const AnglePiToPi yaw = std::atan2(det, dot);
  std::cout << "yaw = " << yaw << std::endl;

  return yaw;
}

void eliminate_yaw(geometry_msgs::msg::Quaternion &q)
{
  const AnglePiToPi yaw = quaternion_to_yaw(q);
  const Eigen::Quaterniond q_yaw_inv(Eigen::AngleAxisd(-yaw.radian(), Eigen::Vector3d::UnitZ()));
  q = message::to_ros(message::from_ros(q) * q_yaw_inv);
}

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
