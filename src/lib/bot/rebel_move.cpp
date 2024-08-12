#include "edu_robot/bot/rebel_move.hpp"

#include <edu_robot/hardware_component_factory.hpp>

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/sensor_imu.hpp>
#include <edu_robot/sensor_point_cloud.hpp>

#include <tf2/LinearMath/Transform.h>
#include <rclcpp/logging.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <memory>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace bot {

static RebelMove::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  RebelMove::Parameter parameter;

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

RebelMove::RebelMove(
  const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns)
  : robot::Robot(robot_name, std::move(hardware_interface), ns)
  , _parameter(get_robot_ros_parameter(*this))
{ }

void RebelMove::initialize(eduart::robot::HardwareComponentFactory& factory)
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

  // Point Cloud Sensors
  constexpr std::array<const char*, 1> point_cloud_name = { "tof_sensor_ring" };
  constexpr std::array<const char*, 1> point_cloud_tf = { "tof_sensor_ring" };
  const std::array<tf2::Transform, 1> point_cloud_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.0))
  };
  
  for (std::size_t i = 0; i < point_cloud_name.size(); ++i) {
    const auto parameter = robot::SensorPointCloud::get_parameter(
      point_cloud_name[i], {}, *this);
    auto hardware_interface = factory.hardware().at("tof_sensor_ring")->cast<robot::SensorPointCloud::SensorInterface>();
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
  imu_parameter.rotated_frame = getFrameIdPrefix() + Robot::_parameter.tf_base_frame;
  imu_parameter = SensorImu::get_parameter("imu", imu_parameter, *this);

  auto imu_sensor = std::make_shared<robot::SensorImu>(
    "imu",
    getFrameIdPrefix() + "imu/base",
    getFrameIdPrefix() + Robot::_parameter.tf_footprint_frame,
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.04)),
    imu_parameter,
    getTfBroadcaster(),
    *this,
    // factory.imuSensorHardware().at("imu")
    factory.hardware().at("imu")->cast<SensorImu::SensorInterface>()
  );
  registerSensor(imu_sensor);
  factory.hardware().at("imu")->cast<SensorImu::SensorInterface>()->initialize(imu_parameter);

  // Set Up Default Drive Kinematic. Needs to be done here, because method can't be called in constructor 
  // of robot base class.
  switchKinematic(DriveKinematic::MECANUM_DRIVE);
}

RebelMove::~RebelMove()
{

}

Eigen::MatrixXf RebelMove::getKinematicMatrix(const DriveKinematic kinematic) const
{
  Eigen::MatrixXf kinematic_matrix;

  if (kinematic == DriveKinematic::SKID_DRIVE) {
    const float l_x = _parameter.skid.length.x;
    const float l_y = _parameter.skid.length.y;
    const float wheel_radius = _parameter.skid.wheel_diameter * 0.5f;
    const float l_squared = l_x * l_x + l_y * l_y;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, 0.0f, l_squared / (2.0f * l_y),
                         1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y),
                        -1.0f, 0.0f, l_squared / (2.0f * l_y);
    kinematic_matrix *= 1.0f / wheel_radius;
  }
  else if (kinematic == DriveKinematic::MECANUM_DRIVE) {
    const float l_x = _parameter.mecanum.length.x;
    const float l_y = _parameter.mecanum.length.y;
    const float wheel_radius = _parameter.mecanum.wheel_diameter * 0.5f;

    kinematic_matrix.resize(4, 3);
    kinematic_matrix <<  1.0f, -1.0f, (l_x + l_y) * 0.5f,
                         1.0f,  1.0f, (l_x + l_y) * 0.5f,
                        -1.0f,  1.0f, (l_x + l_y) * 0.5f,
                        -1.0f, -1.0f, (l_x + l_y) * 0.5f;
    kinematic_matrix *= 1.0f / wheel_radius;    
  }
  else {
    throw std::invalid_argument("RebelMove: given kinematic is not supported.");
  }

  return kinematic_matrix;
}

} // end namespace bot
} // end namespace robot
} // end namespace eduart
