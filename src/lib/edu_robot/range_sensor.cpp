#include "edu_robot/range_sensor.hpp"
#include "edu_robot/sensor.hpp"
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/range__struct.hpp>

namespace eduart {
namespace robot {


static RangeSensor::Parameter get_range_sensor_parameter(
  const std::string& name, const RangeSensor::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  RangeSensor::Parameter parameter;

  ros_node.declare_parameter<float>(name + "/field_of_view", default_parameter.field_of_view);
  ros_node.declare_parameter<float>(name + "/range_min", default_parameter.range_min);
  ros_node.declare_parameter<float>(name + "/range_max", default_parameter.range_max);

  parameter.field_of_view = ros_node.get_parameter(name + "/field_of_view").as_double();
  parameter.range_min = ros_node.get_parameter(name + "/range_min").as_double();
  parameter.range_max = ros_node.get_parameter(name + "/range_max").as_double();

  return parameter;  
}


RangeSensor::RangeSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                         const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
                         std::shared_ptr<HardwareSensorInterface<float>> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(get_range_sensor_parameter(name, parameter, ros_node))
  , _publisher(ros_node.create_publisher<sensor_msgs::msg::Range>(name + "/range", rclcpp::SensorDataQoS()))
  , _clock(ros_node.get_clock())
  , _hardware_interface(std::move(hardware_interface))
{
  _hardware_interface->registerCallbackProcessMeasurementData(
    std::bind(&RangeSensor::processMeasurementData, this, std::placeholders::_1)
  );
}                         

void RangeSensor::processMeasurementData(const float measurement)
{
  sensor_msgs::msg::Range msg;

  msg.header.frame_id = frameId();
  msg.header.stamp    = _clock->now();
  msg.field_of_view   = _parameter.field_of_view;
  msg.max_range       = _parameter.range_max;
  msg.min_range       = _parameter.range_min;
  msg.radiation_type  = msg.INFRARED;
  msg.range           = measurement;

  _publisher->publish(msg);
  sendInputValue(measurement);
}

} // end namespace eduart
} // end namespace robot
