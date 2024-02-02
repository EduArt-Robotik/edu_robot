#include "edu_robot/sensor_point_cloud.hpp"

namespace eduart {
namespace robot {

PointCloudSensor::PointCloudSensor(
  const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
  const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
  std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
{

};

void PointCloudSensor::processMeasurementData(const std::size_t zone_index, const float distance, const float sigma)
{
  
}

diagnostic::Diagnostic PointCloudSensor::processDiagnosticsImpl()
{

}

} // end namespace eduart
} // end namespace robot
