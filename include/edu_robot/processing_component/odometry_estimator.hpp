/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"

#include <edu_robot/angle.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>

namespace eduart {
namespace robot {
namespace processing {

class OdometryEstimator : public ProcessingComponent
{
public:
  struct Parameter {

  };

  OdometryEstimator(const Parameter parameter, rclcpp::Node& ros_node);
  ~OdometryEstimator() override = default;

  nav_msgs::msg::Odometry processOdometryMessage(
    const std::string& robot_base_frame, const std::string& odom_frame, const Eigen::Vector3f& measured_velocity);

private:
  AnglePiToPi _orientation;
  float _position_x;
  float _position_y;
};

} // end namespace processing
} // end namespace robot
} // end namespace eduart
