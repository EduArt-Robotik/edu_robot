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
  ros_node.declare_parameter<float>("skid.length.x", parameter.skid.length.x);
  ros_node.declare_parameter<float>("skid.length.y", parameter.skid.length.y);
  ros_node.declare_parameter<float>("skid.wheel_diameter", parameter.skid.wheel_diameter);

  // Reading Parameters
  parameter.skid.length.x = ros_node.get_parameter("skid.length.x").as_double();
  parameter.skid.length.y = ros_node.get_parameter("skid.length.y").as_double();
  parameter.skid.wheel_diameter = ros_node.get_parameter("skid.wheel_diameter").as_double();

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
  constexpr std::array<const char*, 6> motor_controller_name = {
    "motor_a", "motor_b", "motor_c", "motor_d", "motor_e", "motor_f"};
  // \todo fix the wrong order of joints!
  constexpr std::array<const char*, 6> motor_controller_joint_name = {
    "base_to_wheel_rear_right", "base_to_wheel_front_right", "base_to_wheel_rear_left",
    "base_to_wheel_front_left", "base_to_wheel_middle_right", "base_to_wheel_middle_left" };

  for (std::size_t i = 0; i < motor_controller_name.size(); ++i) {
    const auto motor_controller_parameter = robot::MotorController::get_parameter(
      motor_controller_name[i], motor_controller_default_parameter, *this
    );
    registerMotorController(std::make_shared<robot::MotorController>(
      motor_controller_name[i],
      i,
      motor_controller_parameter,
      motor_controller_joint_name[i],
      *this,
      factory.motorControllerHardware().at(motor_controller_name[i]),
      factory.motorSensorHardware().at(motor_controller_name[i])
    ));
    factory.motorControllerHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
    factory.motorSensorHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
  }

  // IMU Sensor
  ImuSensor::Parameter imu_parameter;
  imu_parameter.raw_data_mode = false;
  imu_parameter.rotated_frame = Robot::_parameter.tf_base_frame;
  imu_parameter = ImuSensor::get_parameter("imu", imu_parameter, *this);
  
  auto imu_sensor = std::make_shared<robot::ImuSensor>(
    "imu",
    getFrameIdPrefix() + "imu/base",
    getFrameIdPrefix() + Robot::_parameter.tf_footprint_frame,
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.1)),
    imu_parameter,
    getTfBroadcaster(),
    *this,
    factory.imuSensorHardware().at("imu")
  );
  registerSensor(imu_sensor);
  factory.imuSensorHardware().at("imu")->initialize(imu_parameter);

  // Set Up Default Drive Kinematic. Needs to be done here, because method can't be called in constructor 
  // of robot base class.
  // \todo maybe introduce an initialize method that can be called after construction of robot class.
  _kinematic_matrix = getKinematicMatrix(Mode::SKID_DRIVE);
  _inverse_kinematic_matrix = _kinematic_matrix.completeOrthogonalDecomposition().pseudoInverse(); 
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

    kinematic_matrix.resize(6, 3);
    kinematic_matrix <<  1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y),
                         1.0f, 0.0f, l_y / 2.0f,
                        -1.0f, 0.0f, l_y / 2.0f,
                         1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y);
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
