#pragma once

#include "edu_robot/msg/mode.hpp"

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

#include <memory>

namespace eduart {
namespace robot {

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

} // end namespace robot
} // end namespace eduart
