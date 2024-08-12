#include "edu_robot/bot/universal_bot.hpp"

#include <edu_robot/hardware_component_factory.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/robot.hpp>
#include <edu_robot/sensor_range.hpp>
#include <edu_robot/sensor_imu.hpp>
#include <edu_robot/sensor_point_cloud.hpp>

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/src/Core/Matrix.h>

namespace eduart {
namespace robot {
namespace bot {

static UniversalBot::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  UniversalBot::Parameter parameter;

  // Requesting Number of Axes
  ros_node.declare_parameter<int>("number_of_axes", 0);
  parameter.axis.resize(ros_node.get_parameter("number_of_axes").as_int());

  for (std::size_t i = 0; i < parameter.axis.size(); ++i) {
    // Declaring of Parameters
    const std::string prefix = std::string("axis_") + std::to_string(i + 1);
    ros_node.declare_parameter<float>(prefix + ".skid.length.x", parameter.axis[i].length.x);
    ros_node.declare_parameter<float>(prefix + ".skid.length.y", parameter.axis[i].length.y);
    ros_node.declare_parameter<float>(prefix + ".skid.wheel_diameter", parameter.axis[i].wheel_diameter);

    // Reading Parameters
    parameter.axis[i].length.x = ros_node.get_parameter(prefix + ".skid.length.x").as_double();
    parameter.axis[i].length.y = ros_node.get_parameter(prefix + ".skid.length.y").as_double();
    parameter.axis[i].wheel_diameter = ros_node.get_parameter(prefix + ".skid.wheel_diameter").as_double();
  }

  // Requesting Number of Pointcloud Sensors
  ros_node.declare_parameter<int>("point_cloud_sensor.number_of", parameter.number_of_point_cloud_sensors());
  parameter.point_cloud_sensor.resize(ros_node.get_parameter("point_cloud_sensor.number_of").as_int());

  for (std::size_t i = 0; i < parameter.number_of_point_cloud_sensors(); ++i) {
    // Declaring of Parameters
    const std::string prefix = std::string("point_cloud_sensor_") + std::to_string(i);
    ros_node.declare_parameter<std::string>(prefix + ".name", prefix);

    parameter.point_cloud_sensor[i].name = ros_node.get_parameter(prefix + ".name").as_string();
    parameter.point_cloud_sensor[i].transform = Sensor::get_transform_from_parameter(prefix, ros_node);
  }

  return parameter;
}

UniversalBot::UniversalBot(const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface)
  : robot::Robot(robot_name, std::move(hardware_interface))
  , _parameter(get_robot_ros_parameter(*this))
{ }

void UniversalBot::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // Motor Controllers
  std::vector<std::string> motor_name = {
    "motor_a", "motor_b", "motor_c", "motor_d", "motor_e", "motor_f", "motor_g", "motor_h"};
  std::vector<std::string> motor_joint_name = {
    "base_to_motor_1", "base_to_motor_2", "base_to_motor_3", "base_to_motor_4",
    "base_to_motor_5", "base_to_motor_6", "base_to_motor_7", "base_to_motor_8"};
  
  // Create motors and motor controllers.
  RCLCPP_INFO(get_logger(), "creating %u motors", static_cast<unsigned int>(_parameter.axis.size()));
  motor_name.resize(_parameter.axis.size() * 2);
  motor_joint_name.resize(_parameter.axis.size() * 2);

  auto motor_controllers = helper_create_motor_controller(
    factory, motor_name, motor_joint_name, getFrameIdPrefix(), *this
  );

  for (auto& motor_controller : motor_controllers) {
    registerMotorController(motor_controller);
  }

  // Point Cloud Sensors
  constexpr std::array<const char*, 1> point_cloud_name = { "pointcloud_left" };
  constexpr std::array<const char*, 1> point_cloud_tf = { "pointcloud/left" };
  const std::array<tf2::Transform, 1> point_cloud_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.0))
  };
  
  for (std::size_t i = 0; i < point_cloud_name.size(); ++i) {
    const auto parameter = robot::SensorPointCloud::get_parameter(
      point_cloud_name[i], {}, *this);
    auto hardware_interface = factory.hardware().at("pointcloud_left")->cast<robot::SensorPointCloud::SensorInterface>();
    auto point_cloud_sensor = std::make_shared<robot::SensorPointCloud>(
      point_cloud_name[i],
      getFrameIdPrefix() + point_cloud_tf[i],
      getFrameIdPrefix() + Robot::_parameter.tf_base_frame,
      point_cloud_pose[i],
      parameter,
      *this,
      hardware_interface
    );
    registerSensor(point_cloud_sensor);
    hardware_interface->initialize(parameter);
  }

  // IMU Sensor
  SensorImu::Parameter imu_parameter;
  imu_parameter.raw_data_mode = false;
  imu_parameter.rotated_frame = Robot::_parameter.tf_base_frame;
  imu_parameter = SensorImu::get_parameter("imu", imu_parameter, *this);
  
  auto imu_sensor = std::make_shared<robot::SensorImu>(
    "imu",
    getFrameIdPrefix() + "imu/base",
    getFrameIdPrefix() + Robot::_parameter.tf_footprint_frame,
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.1)),
    imu_parameter,
    getTfBroadcaster(),
    *this,
    factory.hardware().at("imu")->cast<robot::SensorImu::SensorInterface>()
  );
  registerSensor(imu_sensor);
  factory.hardware().at("imu")->cast<robot::SensorImu::SensorInterface>()->initialize(imu_parameter);

  // Set Up Default Drive Kinematic. Needs to be done here, because method can't be called in constructor 
  // of robot base class.
  // \todo maybe introduce an initialize method that can be called after construction of robot class.
  _kinematic_matrix = getKinematicMatrix(DriveKinematic::SKID_DRIVE);
  _inverse_kinematic_matrix = _kinematic_matrix.completeOrthogonalDecomposition().pseudoInverse();
}

UniversalBot::~UniversalBot()
{

}

Eigen::MatrixXf UniversalBot::getKinematicMatrix(const DriveKinematic kinematic) const
{
  Eigen::MatrixXf kinematic_matrix;

  if (kinematic == DriveKinematic::SKID_DRIVE) {
    kinematic_matrix.resize(_parameter.axis.size() * 2, 3);

    for (std::size_t axis = 0; axis < _parameter.axis.size(); ++axis) {
      const float l_x = _parameter.axis[axis].length.x;
      const float l_y = _parameter.axis[axis].length.y;
      const float wheel_radius = _parameter.axis[axis].wheel_diameter * 0.5f;
      const float l_squared = l_x * l_x + l_y * l_y;

      kinematic_matrix.row(axis * 2 + 0) = Eigen::Vector3f(-1.0f, 0.0f, l_squared / (2.0f * l_y)) / wheel_radius;
      kinematic_matrix.row(axis * 2 + 1) = Eigen::Vector3f( 1.0f, 0.0f, l_squared / (2.0f * l_y)) / wheel_radius;
    }
  }
  else {
    throw std::invalid_argument("Flex Bot: given kinematic is not supported.");
  }

  return kinematic_matrix;
}

} // end namespace bot
} // end namespace robot
} // end namespace eduart
