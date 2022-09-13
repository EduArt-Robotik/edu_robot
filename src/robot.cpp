#include "eduart_robot/robot.hpp"

namespace eduart_robot
{

Robot::Robot(const std::string& robot_name)
  : rclcpp::Node(robot_name)
{

}

Robot::~Robot()
{
}

}  // namespace eduart_robot
