#include "edu_robot/range_sensor.hpp"
#include "edu_robot/sensor.hpp"
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/range__struct.hpp>

namespace eduart {
namespace robot {

RangeSensor::RangeSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                         const tf2::Transform sensor_transform, const Parameter parameter,
                         std::shared_ptr<rclcpp::Node> ros_node)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
  , _publisher(ros_node->create_publisher<sensor_msgs::msg::Range>(name + "/range", rclcpp::SensorDataQoS()))
  , _clock(ros_node->get_clock())
{

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
}

} // end namespace eduart
} // end namespace robot
