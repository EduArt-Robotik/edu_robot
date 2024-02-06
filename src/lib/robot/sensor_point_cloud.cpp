#include "edu_robot/sensor_point_cloud.hpp"

namespace eduart {
namespace robot {

PointCloudSensor::PointCloudSensor(
  const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
  const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
  std::shared_ptr<SensorInterface> hardware_interface)
  : Sensor(name, frame_id, reference_frame_id, sensor_transform)
  , _parameter(parameter)
  , _clock(ros_node.get_clock())
  , _hardware_interface(hardware_interface)
{
  // Preparing Point Cloud
  _point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  _point_cloud->header.frame_id = frame_id;
  _point_cloud->width = _parameter.number_of_zones.horizontal;
  _point_cloud->height = _parameter.number_of_zones.vertical;
  _point_cloud->is_bigendian = false;
  _point_cloud->point_step = 4 * 4; // 4 * float
  _point_cloud->row_step = _point_cloud->point_step * _point_cloud->width;
  _point_cloud->data.resize(_point_cloud->width * _point_cloud->height * _point_cloud->point_step);

  sensor_msgs::msg::PointField point_field;
  point_field.name = "x";
  point_field.offset = 0;
  point_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_field.count = 1;
  _point_cloud->fields.push_back(point_field);

  point_field.name = "y";
  point_field.offset = 4;
  _point_cloud->fields.push_back(point_field);

  point_field.name = "z";
  point_field.offset = 8;
  _point_cloud->fields.push_back(point_field);

  point_field.name = "sigma";
  point_field.offset = 12;
  _point_cloud->fields.push_back(point_field);

  // Preparing Processing Data
  _processing_data.number_of_zones = _parameter.number_of_zones.horizontal * _parameter.number_of_zones.vertical;
  _processing_data.current_zone = 0;
  _processing_data.next_expected_zone = 0;
  _processing_data.tan_x_lookup.resize(_processing_data.number_of_zones);
  _processing_data.tan_y_lookup.resize(_processing_data.number_of_zones);

  const float alpha_increment_vertical = _parameter.fov.vertical / static_cast<double>(_parameter.number_of_zones.vertical);
  const float alpha_increment_horizontal = _parameter.fov.horizontal / static_cast<double>(_parameter.number_of_zones.horizontal);

  for (std::size_t idx_height = 0; idx_height < _parameter.number_of_zones.horizontal; ++idx_height) {
    const int idx_beam_y = ((static_cast<float>(_parameter.number_of_zones.vertical) / 2.0f) - idx_height - 0.5f);

    for (std::size_t idx_width = 0; idx_width < _parameter.number_of_zones.vertical; ++idx_width) {
      const std::size_t idx_zone = idx_height * _parameter.number_of_zones.horizontal + idx_width;
      const int idx_beam_x = ((static_cast<float>(_parameter.number_of_zones.horizontal) / 2.0f) - idx_width - 0.5f);

      _processing_data.tan_x_lookup[idx_zone] = std::tan(idx_beam_x * alpha_increment_horizontal);
      _processing_data.tan_x_lookup[idx_zone] = std::tan(idx_beam_y * alpha_increment_vertical);
    }
  }

  // Publisher
  _publisher = std::make_shared<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(
    name + "/point_cloud", rclcpp::SensorDataQoS()
  );

  // Registering Processing Data Callback
  _hardware_interface->registerCallbackProcessMeasurementData(std::bind(
    &PointCloudSensor::processMeasurementData, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
  );
};

void PointCloudSensor::processMeasurementData(const std::size_t zone_index, const float distance, const float sigma)
{
  if (zone_index != _processing_data.next_expected_zone) {
    RCLCPP_WARN(
      rclcpp::get_logger("PointCloudSensor"),
      "zone_index %u is not expected one %u.",
      static_cast<unsigned int>(zone_index),
      static_cast<unsigned int>(_processing_data.next_expected_zone)
    );
  }

  const std::size_t idx_point_x = zone_index * _point_cloud->point_step + _point_cloud->fields[0].offset;
  const std::size_t idx_point_y = zone_index * _point_cloud->point_step + _point_cloud->fields[1].offset;
  const std::size_t idx_point_z = zone_index * _point_cloud->point_step + _point_cloud->fields[2].offset;
  const std::size_t idx_sigma = zone_index * _point_cloud->point_step + _point_cloud->fields[3].offset;

  _point_cloud->data[idx_point_x] = _processing_data.tan_x_lookup[zone_index] * distance;
  _point_cloud->data[idx_point_y] = _processing_data.tan_y_lookup[zone_index] * distance;
  _point_cloud->data[idx_point_z] = distance;
  _point_cloud->data[idx_sigma] = sigma;

  _processing_data.current_zone = zone_index;

  if (zone_index + 1 < _processing_data.number_of_zones) {
    // measurement not complete yet
    _processing_data.next_expected_zone = zone_index + 1;
    return;
  }

  // measurement finished --> publish point cloud
  _processing_data.next_expected_zone = 0;
  _publisher->publish(*_point_cloud);
}

diagnostic::Diagnostic PointCloudSensor::processDiagnosticsImpl()
{
  return { };
}

} // end namespace eduart
} // end namespace robot
