#include "edu_robot/hardware/can_gateway/sensor_point_cloud_fusion.hpp"
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

SensorPointCloudFusion::SensorPointCloudFusion(std::vector<std::shared_ptr<SensorPointCloud::SensorInterface>> sensor_hardware)
  : _sensor_hardware(sensor_hardware)
  , _received_point_cloud(_sensor_hardware.size(), false)
  , _point_cloud(std::make_shared<sensor_msgs::msg::PointCloud2>())
{
  for (std::size_t i = 0; i < _sensor_hardware.size(); ++i) {
    _sensor_hardware[i]->registerCallbackProcessMeasurementData(
      std::bind(&SensorPointCloudFusion::processPointcloudMeasurement, this, std::placeholders::_1, i)
    );
  }
}

void SensorPointCloudFusion::initialize(const SensorPointCloud::Parameter& parameter)
{
  for (auto& sensor : _sensor_hardware) {
    sensor->initialize(parameter);
  }
}

void SensorPointCloudFusion::processPointcloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index)
{
  std::scoped_lock lock(_data_mutex);

  // handle sensor index first
  if (sensor_index >= _sensor_hardware.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("SensorPointCloudFusion"), "given sensor index is out of range.");
    throw std::invalid_argument("SensorPointCloudFusion: given sensor index is out of range.");
  }
  _received_point_cloud[sensor_index] = true;

  // take fields if no one are set up
  if (_point_cloud->fields.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("SensorPointCloudFusion"), "taking point cloud message fields from given point cloud.");
    _point_cloud->fields = point_cloud.fields;
    _point_cloud->point_step = point_cloud.point_step;
  }
  // only take point cloud if contains same fields
  if (_point_cloud->fields != point_cloud.fields || _point_cloud->point_step != point_cloud.point_step) {
    RCLCPP_ERROR(rclcpp::get_logger("SensorPointCloudFusion"), "given point cloud doesn't fit to previous ones. --> skip point cloud");
    throw std::invalid_argument("SensorPointCloudFusion: given point cloud doesn't fit to previous ones.");
  }

  // merging point clouds
  // taking header from last point cloud
  _point_cloud->header = point_cloud.header;
  // general settings
  _point_cloud->is_bigendian = false;
  _point_cloud->is_dense = false;
  // put all points in a single row, because they are not sorted
  _point_cloud->height = 1;
  _point_cloud->width += point_cloud.height * point_cloud.width;
  // calculate new row step
  _point_cloud->row_step = _point_cloud->point_step * _point_cloud->width;
  // copying point data
  _point_cloud->data.insert(_point_cloud->data.begin(), point_cloud.data.begin(), point_cloud.data.end());

  // checking if point cloud is complete
  for (const auto received : _received_point_cloud) {
    if (received == false) {
      // is not complete --> return and process next point cloud
      return;
    }
  }

  // else: point cloud is complete --> send and reset it
  _callback_process_measurement(*_point_cloud);
  _point_cloud->height = 0;
  _point_cloud->width = 0;
  _point_cloud->data.clear();
  std::fill(_received_point_cloud.begin(), _received_point_cloud.end(), false);
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
