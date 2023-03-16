#include "edu_robot/processing_component/odometry_estimator.hpp"
#include "edu_robot/processing_component/processing_component.hpp"

#include <Eigen/Geometry> 

namespace eduart {
namespace robot {
namespace processing {

OdometryEstimator::OdometryEstimator(const Parameter parameter, rclcpp::Node& ros_node)
  : ProcessingComponent("odometry_estimator", ros_node)
{
  (void)parameter;
  _orientation.setRadian(0.0f);
  _position_x = 0.0f;
  _position_y = 0.0f;
}

nav_msgs::msg::Odometry OdometryEstimator::processOdometryMessage(
    const std::string& robot_base_frame, const Eigen::Vector3f& measured_velocity)
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


  // Constructing Message
  nav_msgs::msg::Odometry odometry_msg;

  odometry_msg.header.stamp = _clock->now();
  odometry_msg.header.frame_id = "odom";
  odometry_msg.child_frame_id = robot_base_frame;

  // Twist Part
  odometry_msg.twist.twist.linear.x = measured_velocity.x();
  odometry_msg.twist.twist.linear.y = measured_velocity.y();
  odometry_msg.twist.twist.linear.z = 0.0;

  odometry_msg.twist.twist.angular.x = 0.0;
  odometry_msg.twist.twist.angular.y = 0.0;
  odometry_msg.twist.twist.angular.z = measured_velocity.z();

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

} // end namespace processing
} // end namespace robot
} // end namespace eduart
