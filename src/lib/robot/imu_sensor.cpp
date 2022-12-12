#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/sensor.hpp"

#include <functional>
#include <memory>

#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {

static ImuSensor::Parameter get_imu_sensor_parameter(
  const std::string sensor_name, const ImuSensor::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  ImuSensor::Parameter parameter;

  ros_node.declare_parameter<bool>(sensor_name + "/raw_data_mode", default_parameter.raw_data_mode);
  ros_node.declare_parameter<std::string>(sensor_name + "/tf_frame_rotated", default_parameter.rotated_frame);

  parameter.raw_data_mode = ros_node.get_parameter(sensor_name + "/raw_data_mode").as_bool();
  parameter.rotated_frame = ros_node.get_parameter(sensor_name + "/tf_frame_rotated").as_string();

  return parameter;
}

ImuSensor::ImuSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                     const tf2::Transform sensor_transform, const Parameter parameter,
                     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, rclcpp::Node& ros_node,
                     std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(get_imu_sensor_parameter(name, parameter, ros_node))
  , _tf_broadcaster(tf_broadcaster)
  , _clock(ros_node.get_clock())
  , _hardware_interface(std::move(hardware_interface))
{
  _hardware_interface->registerCallbackProcessMeasurementData(
    std::bind(&ImuSensor::processMeasurementData, this, std::placeholders::_1)
  );
}                     

void ImuSensor::processMeasurementData(const Eigen::Quaterniond& measurement)
{
  geometry_msgs::msg::TransformStamped msg;

  msg.header.frame_id = frameId();
  msg.header.stamp    = _clock->now();
  msg.child_frame_id  = _parameter.rotated_frame;
  
  msg.transform.translation.x = 0.0;
  msg.transform.translation.y = 0.0;
  msg.transform.translation.z = 0.0;

  msg.transform.rotation.x = measurement.x();
  msg.transform.rotation.y = measurement.y();
  msg.transform.rotation.z = measurement.z();
  msg.transform.rotation.w = measurement.w();

  _tf_broadcaster->sendTransform(msg);
}

} // end namespace eduart
} // end namespace robot
