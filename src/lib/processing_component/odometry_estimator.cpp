#include "edu_robot/processing_component/odometry_estimator.hpp"
#include "edu_robot/processing_component/processing_component.hpp"

#include <Eigen/Geometry> 
#include <tf2/LinearMath/Transform.h>

namespace eduart {
namespace robot {
namespace processing {

OdometryEstimator::OdometryEstimator(const Parameter parameter, rclcpp::Node& ros_node)
  : ProcessingComponent("odometry_estimator", ros_node)
{
  (void)parameter;
}

void OdometryEstimator::process(const Eigen::Vector3f& measured_velocity)
{
  // Estimate delta t
  const auto now = _clock->now();
  const double dt = (now - _last_processing).seconds();
  _last_processing = now;

  // Processing Velocity
  const Eigen::Vector2f liner_velocity(measured_velocity.x(), measured_velocity.y());
  const Eigen::Vector2f direction = Eigen::Rotation2Df(_orientation) * liner_velocity;

  _orientation += measured_velocity.z() * dt;
  _position_x  += direction.x() * dt;
  _position_y  += direction.y() * dt;

  _linear_velocity_x = measured_velocity.x();
  _linear_velocity_y = measured_velocity.y();
  _angular_velocity_z = measured_velocity.z();
}

nav_msgs::msg::Odometry OdometryEstimator::getOdometryMessage(const std::string& robot_base_frame, const std::string& odom_frame) const
{
  // Constructing Message
  nav_msgs::msg::Odometry odometry_msg;

  odometry_msg.header.stamp = _last_processing;
  odometry_msg.header.frame_id = odom_frame;
  odometry_msg.child_frame_id = robot_base_frame;

  // Twist Part
  odometry_msg.twist.twist.linear.x = _linear_velocity_x;
  odometry_msg.twist.twist.linear.y = _linear_velocity_y;
  odometry_msg.twist.twist.linear.z = 0.0;

  odometry_msg.twist.twist.angular.x = 0.0;
  odometry_msg.twist.twist.angular.y = 0.0;
  odometry_msg.twist.twist.angular.z = _angular_velocity_z;

  odometry_msg.twist.covariance.fill(0.0);

  // Pose Part
  const Eigen::Quaternionf q_orientation(Eigen::AngleAxisf(_orientation, Eigen::Vector3f::UnitZ()));
  odometry_msg.pose.pose.orientation.w = q_orientation.w();
  odometry_msg.pose.pose.orientation.x = q_orientation.x();
  odometry_msg.pose.pose.orientation.y = q_orientation.y();
  odometry_msg.pose.pose.orientation.z = q_orientation.z();

  odometry_msg.pose.pose.position.x = _position_x;
  odometry_msg.pose.pose.position.y = _position_y;
  odometry_msg.pose.pose.position.z = 0.0;

  odometry_msg.pose.covariance.fill(0.0);

  return odometry_msg;
}

geometry_msgs::msg::TransformStamped OdometryEstimator::getTfMessage(const std::string& robot_base_frame, const std::string& odom_frame) const
{
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.frame_id = odom_frame;
  tf_msg.header.stamp    = _last_processing;
  tf_msg.child_frame_id  = robot_base_frame;
  
  tf_msg.transform.translation.x = _position_x;
  tf_msg.transform.translation.y = _position_y;
  tf_msg.transform.translation.z = 0.0;

  const Eigen::Quaternionf q_orientation(Eigen::AngleAxisf(_orientation, Eigen::Vector3f::UnitZ()));
  tf_msg.transform.rotation.x = q_orientation.x();
  tf_msg.transform.rotation.y = q_orientation.y();
  tf_msg.transform.rotation.z = q_orientation.z();
  tf_msg.transform.rotation.w = q_orientation.w();

  return tf_msg;
}

} // end namespace processing
} // end namespace robot
} // end namespace eduart
