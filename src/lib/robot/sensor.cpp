#include "edu_robot/sensor.hpp"

namespace eduart {
namespace robot {

tf2::Transform Sensor::get_transform_from_parameter(const std::string& sensor_name, rclcpp::Node& ros_node)
{
  ros_node.declare_parameter<float>(sensor_name + ".transform.translation.x", 0.0f);
  ros_node.declare_parameter<float>(sensor_name + ".transform.translation.y", 0.0f);
  ros_node.declare_parameter<float>(sensor_name + ".transform.translation.z", 0.0f);
  ros_node.declare_parameter<float>(sensor_name + ".transform.orientation.roll", 0.0f);
  ros_node.declare_parameter<float>(sensor_name + ".transform.orientation.pitch", 0.0f);
  ros_node.declare_parameter<float>(sensor_name + ".transform.orientation.yaw", 0.0f);

  const float x = ros_node.get_parameter(sensor_name + ".transform.translation.x").as_double();
  const float y = ros_node.get_parameter(sensor_name + ".transform.translation.y").as_double();
  const float z = ros_node.get_parameter(sensor_name + ".transform.translation.z").as_double();
  const float roll = ros_node.get_parameter(sensor_name + ".transform.orientation.roll").as_double();
  const float pitch = ros_node.get_parameter(sensor_name + ".transform.orientation.pitch").as_double();
  const float yaw = ros_node.get_parameter(sensor_name + ".transform.orientation.yaw").as_double();

  tf2::Vector3 translation(x, y, z);
  tf2::Quaternion orientation;
  orientation.setEuler(yaw, pitch, roll);
  return tf2::Transform(orientation, translation);
}

Sensor::Sensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
               const tf2::Transform sensor_transform)
  : _name(name)
  , _frame_id(frame_id)
  , _reference_frame_id(reference_frame_id)
  , _sensor_transform(sensor_transform)
{

}

geometry_msgs::msg::TransformStamped Sensor::getTransformMsg(const rclcpp::Time stamp) const
{
  geometry_msgs::msg::TransformStamped message;
  
  message.child_frame_id  = _frame_id;
  message.header.frame_id = _reference_frame_id;
  message.header.stamp    = stamp;
  message.transform       = tf2::toMsg(_sensor_transform);

  return message;
}

} // end namespace eduart
} // end namespace robot
