#include "edu_robot/robot.hpp"

namespace eduart {
namespace robot {

Robot::Robot(const std::string& robot_name)
  : rclcpp::Node(robot_name)
{

}

Robot::~Robot()
{
}

} // end namespace robot
} // end namespace eduart
