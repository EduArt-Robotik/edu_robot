#include "edu_robot/motor.hpp"

namespace eduart {
namespace robot {

Motor::Parameter Motor::get_parameter(
  const std::string& name, const Motor::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  std::string prefix = name;
  std::replace(prefix.begin(), prefix.end(), '/', '.');
  Motor::Parameter parameter;

  ros_node.declare_parameter<bool>(prefix + ".inverted", default_parameter.inverted);
  ros_node.declare_parameter<float>(prefix + ".gear_ratio", default_parameter.gear_ratio);
  ros_node.declare_parameter<float>(prefix + ".encoder_ratio", default_parameter.encoder_ratio);
  ros_node.declare_parameter<float>(prefix + ".max_rpm", default_parameter.max_rpm);
  ros_node.declare_parameter<int>(prefix + ".control_frequency", default_parameter.control_frequency);
  ros_node.declare_parameter<int>(prefix + ".timeout_ms", default_parameter.timeout_ms);
  ros_node.declare_parameter<float>(prefix + ".pid.kp", default_parameter.kp);
  ros_node.declare_parameter<float>(prefix + ".pid.ki", default_parameter.ki);
  ros_node.declare_parameter<float>(prefix + ".pid.kd", default_parameter.kd);
  ros_node.declare_parameter<float>(prefix + ".weight_low_pass_set_point", default_parameter.weight_low_pass_set_point);
  ros_node.declare_parameter<float>(prefix + ".weight_low_pass_encoder", default_parameter.weight_low_pass_encoder);
  ros_node.declare_parameter<bool>(prefix + ".encoder_inverted", default_parameter.encoder_inverted);
  ros_node.declare_parameter<bool>(prefix + ".closed_loop", default_parameter.closed_loop);
  ros_node.declare_parameter<int>(prefix + ".index", 0);
  
  parameter.inverted = ros_node.get_parameter(prefix + ".inverted").as_bool();
  parameter.gear_ratio = ros_node.get_parameter(prefix + ".gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(prefix + ".encoder_ratio").as_double();
  parameter.max_rpm = ros_node.get_parameter(prefix + ".max_rpm").as_double();
  parameter.control_frequency = ros_node.get_parameter(prefix + ".control_frequency").as_int();
  parameter.timeout_ms = ros_node.get_parameter(prefix + ".timeout_ms").as_int();
  parameter.kp = ros_node.get_parameter(prefix + ".pid.kp").as_double();
  parameter.ki = ros_node.get_parameter(prefix + ".pid.ki").as_double();
  parameter.kd = ros_node.get_parameter(prefix + ".pid.kd").as_double();
  parameter.weight_low_pass_set_point = ros_node.get_parameter(prefix + ".weight_low_pass_set_point").as_double();
  parameter.weight_low_pass_encoder = ros_node.get_parameter(prefix + ".weight_low_pass_encoder").as_double();
  parameter.encoder_inverted = ros_node.get_parameter(prefix + ".encoder_inverted").as_bool();
  parameter.closed_loop = ros_node.get_parameter(prefix + ".closed_loop").as_bool();
  parameter.index = ros_node.get_parameter(prefix + ".index").as_int();

  return parameter;
}

Motor::Motor(const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, const std::string& urdf_joint_name)
  : _parameter(parameter)
  , _name(name)
  , _urdf_joint_name(urdf_joint_name)
  , _stamp_last_measurement(ros_node.get_clock()->now())
{
  _pub_joint_state = ros_node.create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::QoS(2).reliable().durability_volatile()
  );

}

void Motor::processMeasurementData(const Rpm rpm, const bool enabled_flag, const rclcpp::Time& stamp)
{
  _measured_rpm = rpm;
  _enabled = enabled_flag;

  // \todo Check if calculation is correct! At the moment used for visualization only, so no need for accurate calc...
  // perform wheel position calculation
  const auto delta_t = stamp - _stamp_last_measurement;

  _current_wheel_position += delta_t.seconds() * rpm.radps();
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