#include "edu_robot/motor.hpp"

namespace eduart {
namespace robot {

Motor::Parameter Motor::get_parameter(
  const std::string& name, const Motor::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  std::string prefix = name;
  std::replace(prefix.begin(), prefix.end(), '/', '.');
  Motor::Parameter parameter;

  ros_node.declare_parameter<bool>(prefix + ".closed_loop", default_parameter.closed_loop);
  ros_node.declare_parameter<int>(prefix + ".index", 0);
  ros_node.declare_parameter<float>(name + ".max_rpm", default_parameter.max_rpm);

  ros_node.declare_parameter<float>(prefix + ".pid.kp", default_parameter.kp);
  ros_node.declare_parameter<float>(prefix + ".pid.ki", default_parameter.ki);
  ros_node.declare_parameter<float>(prefix + ".pid.kd", default_parameter.kd);
  
  parameter.closed_loop = ros_node.get_parameter(prefix + ".closed_loop").as_bool();
  parameter.index = ros_node.get_parameter(prefix + ".index").as_int();
  parameter.max_rpm = ros_node.get_parameter(name + ".max_rpm").as_double();

  parameter.kp = ros_node.get_parameter(prefix + ".pid.kp").as_double();
  parameter.ki = ros_node.get_parameter(prefix + ".pid.ki").as_double();
  parameter.kd = ros_node.get_parameter(prefix + ".pid.kd").as_double();

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
  // Note: this method should be thread safe.
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