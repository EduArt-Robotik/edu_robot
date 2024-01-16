#include "edu_robot/sensor_range.hpp"
#include "edu_robot/sensor.hpp"

#include <functional>
#include <memory>

#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>


namespace eduart {
namespace robot {

static SensorRange::Parameter get_range_sensor_parameter(
  const std::string& name, const SensorRange::Parameter& default_parameter, rclcpp::Node& ros_node)
{
  std::string prefix = name;
  std::replace(prefix.begin(), prefix.end(), '/', '.');
  SensorRange::Parameter parameter;

  ros_node.declare_parameter<float>(prefix + ".field_of_view", default_parameter.field_of_view);
  ros_node.declare_parameter<float>(prefix + ".range_min", default_parameter.range_min);
  ros_node.declare_parameter<float>(prefix + ".range_max", default_parameter.range_max);

  parameter.field_of_view = ros_node.get_parameter(prefix + ".field_of_view").as_double();
  parameter.range_min = ros_node.get_parameter(prefix + ".range_min").as_double();
  parameter.range_max = ros_node.get_parameter(prefix + ".range_max").as_double();

  return parameter;  
}


SensorRange::SensorRange(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                         const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
                         std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , processing::ProcessingComponentOutput<float>(name)
  , _parameter(get_range_sensor_parameter(name, parameter, ros_node))
  , _publisher(ros_node.create_publisher<sensor_msgs::msg::Range>(name + "/range", rclcpp::SensorDataQoS()))
  , _clock(ros_node.get_clock())
  , _hardware_interface(std::move(hardware_interface))
  , _last_processing(_clock->now())
  , _processing_dt_statistic(std::make_shared<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>>(
      "processing dt", "ms", 20, 300, 1000, 50, 100)
    )
{
  _hardware_interface->registerCallbackProcessMeasurementData(
    std::bind(&SensorRange::processMeasurementData, this, std::placeholders::_1)
  );
}                         

void SensorRange::processMeasurementData(const float measurement)
{
  // Do statistics for diagnostic
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _last_processing).nanoseconds();
  _processing_dt_statistic->update(dt / 1000000);
  _last_processing = now;

  // Process Sensor Measurement
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

diagnostic::Diagnostic SensorRange::processDiagnosticsImpl()
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
