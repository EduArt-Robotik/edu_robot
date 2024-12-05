#include "edu_robot/sensor_point_cloud.hpp"

namespace eduart {
namespace robot {

SensorPointCloud::Parameter SensorPointCloud::get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  SensorPointCloud::Parameter parameter;

  (void)name;
  (void)ros_node;
  (void)parameter;

  return default_parameter;
}

SensorPointCloud::SensorPointCloud(
  const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
  const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
  std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
  , _clock(ros_node.get_clock())
  , _hardware_interface(hardware_interface)
{
  // Publisher
  _publisher = ros_node.create_publisher<sensor_msgs::msg::PointCloud2>(
    name + "/point_cloud", rclcpp::SensorDataQoS()
  );

  // Registering Processing Data Callback
  _hardware_interface->registerCallbackProcessMeasurementData(std::bind(
    &SensorPointCloud::processMeasurementData, this, std::placeholders::_1)
  );
};

void SensorPointCloud::processMeasurementData(sensor_msgs::msg::PointCloud2& point_cloud)
{
  point_cloud.header.frame_id = frameId();
  _publisher->publish(point_cloud);
}

diagnostic::Diagnostic SensorPointCloud::processDiagnosticsImpl()
{
  return { };
}

} // end namespace eduart
} // end namespace robot
