#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/sensor.hpp"

#include <functional>
#include <memory>

#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {

ImuSensor::Parameter ImuSensor::get_parameter(
  const std::string& sensor_name, const ImuSensor::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  std::string sensor_prefix = sensor_name;
  std::replace(sensor_prefix.begin(), sensor_prefix.end(), '/', '.');
  ImuSensor::Parameter parameter;

  ros_node.declare_parameter<bool>(
    sensor_prefix + ".raw_data_mode", default_parameter.raw_data_mode);
  ros_node.declare_parameter<bool>(sensor_prefix + "publish_tf", default_parameter.publish_tf);
  ros_node.declare_parameter<float>(
    sensor_prefix + ".fusion_weight", default_parameter.fusion_weight);
  ros_node.declare_parameter<float>(
    sensor_prefix + ".mounting_orientation.roll", default_parameter.mount_orientation.roll);
  ros_node.declare_parameter<float>(
    sensor_prefix + ".mounting_orientation.pitch", default_parameter.mount_orientation.pitch);
  ros_node.declare_parameter<float>(
    sensor_prefix + ".mounting_orientation.yaw", default_parameter.mount_orientation.yaw);        
  ros_node.declare_parameter<std::string>(
    sensor_prefix + ".tf_frame_rotated", default_parameter.rotated_frame);

  parameter.raw_data_mode = ros_node.get_parameter(sensor_prefix + ".raw_data_mode").as_bool();
  parameter.publish_tf = ros_node.get_parameter(sensor_prefix + "publish_tf").as_bool();
  parameter.fusion_weight = ros_node.get_parameter(sensor_prefix + ".fusion_weight").as_double();
  parameter.mount_orientation.roll = ros_node.get_parameter(
    sensor_prefix + ".mounting_orientation.roll").as_double();
  parameter.mount_orientation.pitch = ros_node.get_parameter(
    sensor_prefix + ".mounting_orientation.pitch").as_double();
  parameter.mount_orientation.yaw = ros_node.get_parameter(
    sensor_prefix + ".mounting_orientation.yaw").as_double();        
  parameter.rotated_frame = ros_node.get_parameter(sensor_prefix + ".tf_frame_rotated").as_string();

  return parameter;
}

ImuSensor::ImuSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                     const tf2::Transform sensor_transform, const Parameter parameter,
                     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, rclcpp::Node& ros_node,
                     std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
  , _tf_broadcaster(tf_broadcaster)
  , _pub_imu_message(ros_node.create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS()))
  , _clock(ros_node.get_clock())
  , _hardware_interface(std::move(hardware_interface))
{
  _hardware_interface->registerCallbackProcessMeasurementData(std::bind(
    &ImuSensor::processMeasurementData,
    this,
    std::placeholders::_1,
    std::placeholders::_2,
    std::placeholders::_3
  ));
}                     

void ImuSensor::processMeasurementData(
  const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& linear_acceleration)
{
  // Sensor Message
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.frame_id = frameId();
  imu_msg.header.stamp = _clock->now();
  
  imu_msg.orientation.x = orientation.x();
  imu_msg.orientation.y = orientation.y();
  imu_msg.orientation.z = orientation.z();
  imu_msg.orientation.w = orientation.w();

  imu_msg.angular_velocity.x = angular_velocity.x();
  imu_msg.angular_velocity.y = angular_velocity.y();
  imu_msg.angular_velocity.z = angular_velocity.z();

  imu_msg.linear_acceleration.x = linear_acceleration.x();
  imu_msg.linear_acceleration.y = linear_acceleration.y();
  imu_msg.linear_acceleration.z = linear_acceleration.z();

  _pub_imu_message->publish(imu_msg);


  // TF
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.frame_id = frameId();
  tf_msg.header.stamp    = imu_msg.header.stamp;
  tf_msg.child_frame_id  = _parameter.rotated_frame;
  
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;

  if (_parameter.publish_tf) {
    tf_msg.transform.rotation.x = orientation.x();
    tf_msg.transform.rotation.y = orientation.y();
    tf_msg.transform.rotation.z = orientation.z();
    tf_msg.transform.rotation.w = orientation.w();
  }
  else {
    // publishing zero rotation to keep tf tree valid
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  }

  _tf_broadcaster->sendTransform(tf_msg);
}

} // end namespace eduart
} // end namespace robot
