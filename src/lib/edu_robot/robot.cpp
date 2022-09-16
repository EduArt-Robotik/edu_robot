#include "edu_robot/robot.hpp"

namespace eduart {
namespace robot {

Robot::Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
  : rclcpp::Node(robot_name)
  , _hardware_interface(std::move(hardware_interface))
{

}

Robot::~Robot()
{
  
}

} // end namespace robot
} // end namespace eduart
