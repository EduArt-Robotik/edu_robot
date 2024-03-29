#include <edu_robot/bot/turtle/turtle.hpp>
#include <edu_robot/hardware_component_factory.hpp>

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/robot.hpp>
#include <edu_robot/sensor_range.hpp>
#include <edu_robot/sensor_imu.hpp>

#include <tf2/LinearMath/Transform.h>
#include <rclcpp/logging.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <memory>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace turtle {

static Turtle::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  Turtle::Parameter parameter;

  // Declaring of Parameters
  ros_node.declare_parameter<float>("skid.length.x", parameter.skid.length.x);
  ros_node.declare_parameter<float>("skid.length.y", parameter.skid.length.y);
  ros_node.declare_parameter<float>("skid.wheel_diameter", parameter.skid.wheel_diameter);
  
  ros_node.declare_parameter<float>("mecanum.length.x", parameter.mecanum.length.x);
  ros_node.declare_parameter<float>("mecanum.length.y", parameter.mecanum.length.y);
  ros_node.declare_parameter<float>("mecanum.wheel_diameter", parameter.mecanum.wheel_diameter);

  // Reading Parameters
  parameter.skid.length.x = ros_node.get_parameter("skid.length.x").as_double();
  parameter.skid.length.y = ros_node.get_parameter("skid.length.y").as_double();
  parameter.skid.wheel_diameter = ros_node.get_parameter("skid.wheel_diameter").as_double();

  parameter.mecanum.length.x = ros_node.get_parameter("mecanum.length.x").as_double();
  parameter.mecanum.length.y = ros_node.get_parameter("mecanum.length.y").as_double();
  parameter.mecanum.wheel_diameter = ros_node.get_parameter("mecanum.wheel_diameter").as_double();

  return parameter;
}

Turtle::Turtle(
  const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns)
  : robot::Robot(robot_name, std::move(hardware_interface), ns)
  , _parameter(get_robot_ros_parameter(*this))
{ }

void Turtle::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // Motor Controllers
  const std::vector<std::string> motor_name = {
    "motor_a", "motor_b", "motor_c", "motor_d" };
  const std::vector<std::string> motor_joint_name = {
    "base_to_wheel_rear_right", "base_to_wheel_front_right", "base_to_wheel_rear_left", "base_to_wheel_front_left" };
  auto motor_controllers = helper_create_motor_controller(
    factory, motor_name, motor_joint_name, getFrameIdPrefix(), *this
  );

  for (auto& motor_controller : motor_controllers) {
    registerMotorController(motor_controller);
  }

  // Set Up Default Drive Kinematic. Needs to be done here, because method can't be called in constructor 
  // of robot base class.
  switchKinematic(DriveKinematic::MECANUM_DRIVE);
}

Turtle::~Turtle()
{

}

Eigen::MatrixXf Turtle::getKinematicMatrix(const DriveKinematic kinematic) const
{
  Eigen::MatrixXf kinematic_matrix;

  if (kinematic == DriveKinematic::MECANUM_DRIVE) {
    const float l_x = _parameter.mecanum.length.x;
    const float l_y = _parameter.mecanum.length.y;
    const float wheel_radius = _parameter.mecanum.wheel_diameter * 0.5f;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, -1.0f, (l_x + l_y) * 0.5f,
                         1.0f,  1.0f, (l_x + l_y) * 0.5f,
                        -1.0f, -1.0f, (l_x + l_y) * 0.5f,
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f;
    kinematic_matrix *= 1.0f / wheel_radius;    
  }
  else {
    throw std::invalid_argument("Turtle: given kinematic is not supported.");
  }

  return kinematic_matrix;
}

} // end namespace turtle
} // end namespace robot
} // end namespace eduart
