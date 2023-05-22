#include <edu_robot/eduard/eduard.hpp>
#include <edu_robot/hardware_component_factory.hpp>

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/motor_controller.hpp>
#include <edu_robot/robot.hpp>
#include <edu_robot/range_sensor.hpp>
#include <edu_robot/imu_sensor.hpp>

#include <tf2/LinearMath/Transform.h>
#include <rclcpp/logging.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <memory>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace eduard {

static Eduard::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  Eduard::Parameter parameter;

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

Eduard::Eduard(
  const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface, const std::string& ns)
  : robot::Robot(robot_name, std::move(hardware_interface), ns)
  , _parameter(get_robot_ros_parameter(*this))
{ }

void Eduard::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // Lightings
  registerLighting(std::make_shared<robot::Lighting>(
    "head",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.lightingHardware().at("head")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "right_side",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.lightingHardware().at("right_side")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "left_side",
    COLOR::DEFAULT::BACK,
    1.0f,
    factory.lightingHardware().at("left_side")
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "back",
    COLOR::DEFAULT::BACK,
    1.0f,
    factory.lightingHardware().at("back")
  ));

  // Use all representation to set a initial light.
  auto lighting_all = std::make_shared<robot::Lighting>(
    "all",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.lightingHardware().at("all")
  );
  lighting_all->setColor(COLOR::DEFAULT::BACK, Lighting::Mode::RUNNING);
  registerLighting(lighting_all);


  // Motor Controllers
  constexpr robot::MotorController::Parameter motor_controller_default_parameter{ };
  constexpr std::array<const char*, 4> motor_controller_name = {
    "motor_a", "motor_b", "motor_c", "motor_d" };
  constexpr std::array<const char*, 4> motor_controller_joint_name = {
    "base_to_wheel_rear_right", "base_to_wheel_front_right", "base_to_wheel_rear_left", "base_to_wheel_front_left" };

  for (std::size_t i = 0; i < motor_controller_name.size(); ++i) {
    const auto motor_controller_parameter = robot::MotorController::get_parameter(
        motor_controller_name[i], motor_controller_default_parameter, *this
    );

    registerMotorController(std::make_shared<robot::MotorController>(
      motor_controller_name[i],
      i,
      motor_controller_parameter,
      getFrameIdPrefix() + motor_controller_joint_name[i],
      *this,
      factory.motorControllerHardware().at(motor_controller_name[i]),
      factory.motorSensorHardware().at(motor_controller_name[i])
    ));
    factory.motorControllerHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
    factory.motorSensorHardware().at(motor_controller_name[i])->initialize(motor_controller_parameter);
  }


  // Range Sensors
  constexpr std::array<const char*, 4> range_sensor_name = {
    "range/front/left", "range/front/right", "range/rear/left", "range/rear/right" };
  const std::array<tf2::Transform, 4> range_sensor_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17,  0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17, -0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17,  0.063, 0.050)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, -0.063, 0.050))
  };
  constexpr eduart::robot::RangeSensor::Parameter range_sensor_parameter{ 10.0 * M_PI / 180.0, 0.01, 5.0 };

  for (std::size_t i = 0; i < range_sensor_name.size(); ++i) {
    auto range_sensor = std::make_shared<robot::RangeSensor>(
      range_sensor_name[i],
      getFrameIdPrefix() + range_sensor_name[i],
      getFrameIdPrefix() + Robot::_parameter.tf_base_frame,
      range_sensor_pose[i],
      range_sensor_parameter,
      *this,
      factory.rangeSensorHardware().at(range_sensor_name[i])
    );
    registerSensor(range_sensor);
    range_sensor->registerComponentInput(_collision_avoidance_component);
    factory.rangeSensorHardware().at(range_sensor_name[i])->initialize(range_sensor_parameter);
  }

  // IMU Sensor
  ImuSensor::Parameter imu_parameter;
  imu_parameter.raw_data_mode = false;
  imu_parameter.rotated_frame = getFrameIdPrefix() + Robot::_parameter.tf_base_frame;
  imu_parameter = ImuSensor::get_parameter("imu", imu_parameter, *this);

  auto imu_sensor = std::make_shared<robot::ImuSensor>(
    "imu",
    getFrameIdPrefix() + "imu/base",
    getFrameIdPrefix() + Robot::_parameter.tf_footprint_frame,
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.04)),
    imu_parameter,
    getTfBroadcaster(),
    *this,
    factory.imuSensorHardware().at("imu")
  );
  registerSensor(imu_sensor);
  factory.imuSensorHardware().at("imu")->initialize(imu_parameter);

  // Set Up Default Drive Kinematic. Needs to be done here, because method can't be called in constructor 
  // of robot base class.
  switchKinematic(Mode::SKID_DRIVE);
}

Eduard::~Eduard()
{

}

Eigen::MatrixXf Eduard::getKinematicMatrix(const Mode mode) const
{
  Eigen::MatrixXf kinematic_matrix;

  if (mode & Mode::SKID_DRIVE) {
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
  else if (mode & Mode::MECANUM_DRIVE) {
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
    throw std::invalid_argument("Eduard: given kinematic is not supported.");
  }

  return kinematic_matrix;
}

} // end namespace eduard
} // end namespace robot
} // end namespace eduart
