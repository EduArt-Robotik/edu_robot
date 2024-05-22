#include "edu_robot/sensor_imu.hpp"
#include "edu_robot/sensor.hpp"

#include <functional>
#include <memory>

#include <rclcpp/node.hpp>

#include <Eigen/Eigen>

namespace eduart {
namespace robot {

SensorImu::Parameter SensorImu::get_parameter(
  const std::string& sensor_name, const SensorImu::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  std::string sensor_prefix = sensor_name;
  std::replace(sensor_prefix.begin(), sensor_prefix.end(), '/', '.');
  SensorImu::Parameter parameter;

  ros_node.declare_parameter<bool>(
    sensor_prefix + ".raw_data_mode", default_parameter.raw_data_mode);
  ros_node.declare_parameter<bool>(sensor_prefix + ".publish_tf", default_parameter.publish_tf);
  ros_node.declare_parameter<bool>(
    sensor_prefix + ".publish_orientation_without_yaw_tf",
    default_parameter.publish_orientation_without_yaw_tf);
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
  parameter.publish_tf = ros_node.get_parameter(sensor_prefix + ".publish_tf").as_bool();
  parameter.publish_orientation_without_yaw_tf = ros_node.get_parameter(
    sensor_prefix + ".publish_orientation_without_yaw_tf").as_bool();
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

SensorImu::SensorImu(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                     const tf2::Transform sensor_transform, const Parameter parameter,
                     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, rclcpp::Node& ros_node,
                     std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
  , _tf_broadcaster(tf_broadcaster)
  , _pub_imu_message(ros_node.create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS()))
  , _clock(ros_node.get_clock())
  , _hardware_interface(std::move(hardware_interface))
  , _last_processing(_clock->now())
  , _processing_dt_statistic(std::make_shared<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>>(
      "processing dt", "ms", 20, 250, 350, 50, 100)
    )
{
  _hardware_interface->registerCallbackProcessMeasurementData(std::bind(
    &SensorImu::processMeasurementData,
    this,
    std::placeholders::_1,
    std::placeholders::_2,
    std::placeholders::_3
  ));
}                     

void SensorImu::processMeasurementData(
  const Eigen::Quaterniond& orientation, const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& linear_acceleration)
{
  // Note: this method should be thread safe.

  // Do statistics for diagnostic
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _last_processing).nanoseconds();
  _processing_dt_statistic->update(dt / 1000000);
  _last_processing = now;

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
  // Only publish tf if enabled.
  if (_parameter.publish_tf == false) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.frame_id = frameId();
  tf_msg.header.stamp    = imu_msg.header.stamp;
  tf_msg.child_frame_id  = _parameter.rotated_frame;
  
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;

  if (_parameter.publish_orientation_without_yaw_tf) {
    // Estimate orientation from linear acceleration --> without yaw angle.
    const Eigen::Vector3d ground_reference(0.0, 0.0, -9.81);
    const Eigen::Quaterniond q_without_yaw = Eigen::Quaterniond::FromTwoVectors(linear_acceleration, ground_reference);

    tf_msg.transform.rotation.x = q_without_yaw.x();
    tf_msg.transform.rotation.y = q_without_yaw.y();
    tf_msg.transform.rotation.z = q_without_yaw.z();
    tf_msg.transform.rotation.w = q_without_yaw.w();
  }
  else {
    tf_msg.transform.rotation.x = orientation.x();
    tf_msg.transform.rotation.y = orientation.y();
    tf_msg.transform.rotation.z = orientation.z();
    tf_msg.transform.rotation.w = orientation.w();
  }

  _tf_broadcaster->sendTransform(tf_msg);
}

diagnostic::Diagnostic SensorImu::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;

  // processing dt
  if ((_clock->now() - _last_processing).nanoseconds() / 1000000 > _processing_dt_statistic->checkerMean().levelError()) {
    diagnostic.add("processing dt", "timeout", diagnostic::Level::ERROR);
  }
  else {
    diagnostic.add(*_processing_dt_statistic);
  }

  return diagnostic;
}

} // end namespace eduart
} // end namespace robot
