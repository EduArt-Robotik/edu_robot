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
  return std::atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

AnglePiToPi quaternion_to_yaw(const Eigen::Quaterniond& q)
{
  return quaternion_to_yaw(message::to_ros(q));
}

void eliminate_yaw(geometry_msgs::msg::Quaternion &q)
{
  const AnglePiToPi yaw = quaternion_to_yaw(q);
  const Eigen::Quaterniond q_yaw_inv(Eigen::AngleAxisd(-yaw.radian(), Eigen::Vector3d::UnitZ()));
  q = message::to_ros(message::from_ros(q) * q_yaw_inv);
}

void eliminate_yaw(Eigen::Quaterniond& q)
{
  const AnglePiToPi yaw = quaternion_to_yaw(q);
  const Eigen::Quaterniond q_yaw_inv(Eigen::AngleAxisd(-yaw.radian(), Eigen::Vector3d::UnitZ()));
  q = q * q_yaw_inv;  
}

} // end namespace algorithm
} // end namespace robot
} // end namespace eduart
