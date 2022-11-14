#include "edu_robot/motor_controller.hpp"
#include "edu_robot/rotation_per_minute.hpp"
#include <cstdint>
#include <functional>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

namespace eduart {
namespace robot {

MotorController::Parameter get_motor_controller_parameter(
  const std::string& name, const MotorController::Parameter default_parameter, rclcpp::Node& ros_node)
{
  MotorController::Parameter parameter;

  ros_node.declare_parameter<bool>(name + "inverted", default_parameter.inverted);
  ros_node.declare_parameter<float>(name + "/gear_ratio", default_parameter.gear_ratio);
  ros_node.declare_parameter<float>(name + "/encoder_ratio", default_parameter.encoder_ratio);
  ros_node.declare_parameter<float>(name + "/max_rpm", default_parameter.max_rpm);
  ros_node.declare_parameter<int>(name + "/control_frequency", default_parameter.control_frequency);
  ros_node.declare_parameter<float>(name + "/pid/kp", default_parameter.kp);
  ros_node.declare_parameter<float>(name + "/pid/ki", default_parameter.ki);
  ros_node.declare_parameter<float>(name + "/pid/kd", default_parameter.kd);
  ros_node.declare_parameter<float>(name + "/weight_low_pass_set_point", default_parameter.weight_low_pass_set_point);
  ros_node.declare_parameter<float>(name + "/weight_low_pass_encoder", default_parameter.weight_low_pass_encoder);

  parameter.inverted = ros_node.get_parameter(name + "inverted").as_bool();
  parameter.gear_ratio = ros_node.get_parameter(name + "/gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(name + "/encoder_ratio").as_double();
  parameter.max_rpm = ros_node.get_parameter(name + "/max_rpm").as_double();
  parameter.control_frequency = ros_node.get_parameter(name + "/control_frequency").as_int();
  parameter.kp = ros_node.get_parameter(name + "/pid/kp").as_double();
  parameter.ki = ros_node.get_parameter(name + "/pid/ki").as_double();
  parameter.kd = ros_node.get_parameter(name + "/pid/kd").as_double();
  parameter.weight_low_pass_set_point = ros_node.get_parameter(name + "/weight_low_pass_set_point").as_double();
  parameter.weight_low_pass_encoder = ros_node.get_parameter(name + "/weight_low_pass_encoder").as_double();

  return parameter;
}

MotorController::MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                                 const std::string& urdf_joint_name, rclcpp::Node& ros_node,
                                 std::shared_ptr<HardwareComponentInterface<Rpm>> hardware_component_interface,
                                 std::shared_ptr<HardwareSensorInterface<Rpm>> hardware_sensor_interface)
  : _parameter(get_motor_controller_parameter(name, parameter, ros_node))
  , _name(name)
  , _id(id)
  , _urdf_joint_name(urdf_joint_name)
  , _clock(ros_node.get_clock())
  , _stamp_last_measurement(_clock->now())
  , _current_wheel_position(0.0)
  , _hardware_component_interface(hardware_component_interface)
  , _hardware_sensor_interface(hardware_sensor_interface)
{
  _hardware_sensor_interface->registerCallbackProcessMeasurementData(
    std::bind(&MotorController::processMeasurementData, this, std::placeholders::_1)
  );
  _pub_joint_state = ros_node.create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(2).reliable().durability_volatile()
  );
}

MotorController::~MotorController()
{

}

void MotorController::setRpm(const Rpm rpm)
{
  _hardware_component_interface->processSetValue(rpm);
  _set_rpm = rpm;
}

void MotorController::processMeasurementData(const Rpm rpm)
{
  // \todo Check if calculation is correct! At the moment used for visualization only, so no need for accurate calc...
  // perform wheel position calculation
  const auto stamp = _clock->now();
  const auto delta_t = stamp - _stamp_last_measurement;

  _current_wheel_position += delta_t.seconds() * rpm.radps();
  _measured_rpm = rpm;
  _stamp_last_measurement = stamp;

  // publish wheel position as tf message
  // geometry_msgs::msg::TransformStamped msg;

  // publish wheel speed using joint state message
  sensor_msgs::msg::JointState joint_state_msg;

  joint_state_msg.header.frame_id = "";
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.name.push_back(_urdf_joint_name);
  joint_state_msg.velocity.push_back(rpm.radps());
  joint_state_msg.position.push_back(_current_wheel_position);

  _pub_joint_state->publish(joint_state_msg);
}

} // end namespace robot
} // end namespace eduart
