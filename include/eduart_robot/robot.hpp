#pragma once

#include "eduart_robot/visibility_control.h"
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eduart_robot/msg/mode.hpp>
#include <rclcpp/publisher.hpp>

namespace eduart_robot
{

class Robot : public rclcpp::Node
{
protected:
  Robot(const std::string& robot_name);

public:
  virtual ~Robot();

  void callbackVelocity(const std::shared_ptr<const geometry_msgs::msg::Twist>& twist_msgs);
  // void callbackServiceSetMode(const std::shared_ptr<typename Tp>)

private:
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _pub_odometry;
};

}  // namespace eduart_robot
