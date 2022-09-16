/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/msg/detail/set_lighting_color__struct.hpp"
#include "edu_robot/msg/mode.hpp"
#include "edu_robot/msg/set_lighting_color.hpp"
#include "edu_robot/msg/robot_status_report.hpp"
#include "edu_robot/srv/detail/set_mode__struct.hpp"
#include "edu_robot/srv/set_mode.hpp"

#include "edu_robot/robot_hardware_interface.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <map>
#include <string>

namespace eduart {
namespace robot {

class Lighting;

/**
 * \brief Defines and implements basic functionality of robot in Eduart universe. Each robot inherits from this
 *        class has to implement a hardware driver that is used for hardware abstraction.
 */
class Robot : public rclcpp::Node
{
protected:
  Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface);

public:
  virtual ~Robot();

  /**
   * \brief Callback used to process received twist messages by passing it to the hardware abstraction layer.
   *
   * \param twist_msg Received twist message.
   */
  void callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg);
  /**
   * \brief Callback used to set the color and brightness of connected lightings.
   *
   * \param msg Received set lighting color message.
   */
  void callbackSetLightingColor(std::shared_ptr<const edu_robot::msg::SetLightingColor> msg);
  /**
   * \brief Callback to process a set mode request. If the request was accepted a OK state will be responded.
   *        If it wasn't accepted the error state will be responded.
   *
   * \param request The request mode.
   * \param response The response to the send request.
   */
  void callbackServiceSetMode(const std::shared_ptr<edu_robot::srv::SetMode::Request> request,
                              std::shared_ptr<edu_robot::srv::SetMode::Response> response);

protected:
  void registerLighting(std::shared_ptr<Lighting> lighting);

  std::shared_ptr<RobotHardwareInterface> _hardware_interface;

private:
  void handleStatusReport();

  // ROS related members
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _pub_odometry;
  std::shared_ptr<rclcpp::Publisher<edu_robot::msg::RobotStatusReport>> _pub_status_report;
  std::shared_ptr<rclcpp::Service<edu_robot::srv::SetMode>> _srv_set_mode;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist;
  std::shared_ptr<rclcpp::Subscription<edu_robot::msg::SetLightingColor>> _sub_set_lighting_color;

  // Mounted components that are controlled by the hardware interface.
  std::map<std::string, std::shared_ptr<Lighting>> _lightings;
};

} // end namespace robot
} // end namespace eduart
