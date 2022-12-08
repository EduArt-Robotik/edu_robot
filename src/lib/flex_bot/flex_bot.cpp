#include <edu_robot/flex_bot/flex_bot.hpp>
#include <edu_robot/hardware_component_factory.hpp>

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/robot.hpp>
#include <edu_robot/range_sensor.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/src/Core/Matrix.h>

namespace eduart {
namespace robot {
namespace flex_bot {

static FlexBot::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  FlexBot::Parameter parameter;

  // Declaring of Parameters
  ros_node.declare_parameter<std::string>("tf_footprint_frame", parameter.tf_footprint_frame);

  ros_node.declare_parameter<float>("skid/length/x", parameter.skid.length.x);
  ros_node.declare_parameter<float>("skid/length/y", parameter.skid.length.y);
  ros_node.declare_parameter<float>("skid/wheel_diameter", parameter.skid.wheel_diameter);
  
  ros_node.declare_parameter<float>("mecanum/length/x", parameter.mecanum.length.x);
  ros_node.declare_parameter<float>("mecanum/length/y", parameter.mecanum.length.y);
  ros_node.declare_parameter<float>("mecanum/wheel_diameter", parameter.mecanum.wheel_diameter);

  // Reading Parameters
  parameter.tf_footprint_frame = ros_node.get_parameter("tf_footprint_frame").as_string();

  parameter.skid.length.x = ros_node.get_parameter("skid/length/x").as_double();
  parameter.skid.length.y = ros_node.get_parameter("skid/length/y").as_double();
  parameter.skid.wheel_diameter = ros_node.get_parameter("skid/wheel_diameter").as_double();

  parameter.mecanum.length.x = ros_node.get_parameter("mecanum/length/x").as_double();
  parameter.mecanum.length.y = ros_node.get_parameter("mecanum/length/y").as_double();
  parameter.mecanum.wheel_diameter = ros_node.get_parameter("mecanum/wheel_diameter").as_double();

  return parameter;
}

FlexBot::FlexBot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
  : robot::Robot(robot_name, std::move(hardware_interface))
  , _parameter(get_robot_ros_parameter(*this))
{ }

void FlexBot::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // Motor Controllers
  constexpr robot::MotorController::Parameter motor_controller_default_parameter{ };
  constexpr std::array<const char*, 4> motor_controller_name = {
    "motor_a", "motor_b", "motor_c", "motor_d" };
  constexpr std::array<const char*, 4> motor_controller_joint_name = {
    "base_to_wheel_rear_right", "base_to_wheel_front_right", "base_to_wheel_rear_left", "base_to_wheel_front_left" };

  for (std::size_t i = 0; i < motor_controller_name.size(); ++i) {
    registerMotorController(std::make_shared<robot::MotorController>(
      motor_controller_name[i],
      i,
      robot::MotorController::get_motor_controller_parameter(
        motor_controller_name[i], motor_controller_default_parameter, *this
      ),
      motor_controller_joint_name[i],
      *this,
      factory.motorControllerHardware().at(motor_controller_name[i]),
      factory.motorSensorHardware().at(motor_controller_name[i])
    ));
  }

  // Set Up Default Drive Kinematic
  _kinematic_matrix = getKinematicMatrix(Mode::SKID_DRIVE);
}

FlexBot::~FlexBot()
{

}

Eigen::MatrixXf FlexBot::getKinematicMatrix(const Mode mode) const
{
  Eigen::MatrixXf kinematic_matrix;

  if (mode & Mode::SKID_DRIVE) {
    const float l_x = _parameter.skid.length.x;
    const float l_y = _parameter.skid.length.y;
    const float wheel_diameter = _parameter.skid.wheel_diameter;
    const float l_squared = l_x * l_x + l_y * l_y;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix << -1.0f, 0.0f, -l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, -l_squared / (2.0f * l_y),
                         1.0f, 0.0f, -l_squared / (2.0f * l_y),
                         1.0f, 0.0f, -l_squared / (2.0f * l_y);
    kinematic_matrix *= 1.0f / wheel_diameter;
  }
  else {
    throw std::invalid_argument("Flex Bot: given kinematic is not supported.");
  }

  return kinematic_matrix;
}

} // end namespace flex_bot
} // end namespace robot
} // end namespace eduart
