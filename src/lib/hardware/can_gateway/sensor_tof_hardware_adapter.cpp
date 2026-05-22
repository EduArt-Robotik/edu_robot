/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware_adapter.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/measurement/DepthMeasurement.hpp>
#include <sensorring/subscription/Subscription.hpp>

#include <rclcpp/logging.hpp>

#include <functional>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

static std::shared_ptr<sensor_msgs::msg::PointCloud2> create_point_cloud()
{
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->height = 1;
  point_cloud->width = 0;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 4u * sizeof(float); // x, y, z, sigma
  point_cloud->row_step = 0u;

  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;

  field.name = "x";     field.offset = 0u;  point_cloud->fields.push_back(field);
  field.name = "y";     field.offset = 4u;  point_cloud->fields.push_back(field);
  field.name = "z";     field.offset = 8u;  point_cloud->fields.push_back(field);
  field.name = "sigma"; field.offset = 12u; point_cloud->fields.push_back(field);

  return point_cloud;
}

SensorTofHardwareAdapter::SensorTofHardwareAdapter(
  std::shared_ptr<sensorring::manager::MeasurementManager> manager,
  unsigned int sensor_index)
  : _manager(std::move(manager))
  , _sensor_index(sensor_index)
  , _depth_sub(std::make_unique<sensorring::subscription::Subscription>())
  , _point_cloud(create_point_cloud())
{ }

void SensorTofHardwareAdapter::initialize(const SensorPointCloud::Parameter& parameter)
{
  (void)parameter;

  auto depth_group = _manager->depthSensors();

  if (_sensor_index >= depth_group.size()) {
    throw std::out_of_range(
      "SensorTofHardwareAdapter: sensor_index " + std::to_string(_sensor_index) +
      " is out of range (available: " + std::to_string(depth_group.size()) + ")"
    );
  }

  *_depth_sub = depth_group[_sensor_index].subscribe(
    std::bind(&SensorTofHardwareAdapter::onDepthMeasurement, this, std::placeholders::_1)
  );
}

void SensorTofHardwareAdapter::onDepthMeasurement(const sensorring::measurement::DepthMeasurement& m)
{
  std::scoped_lock lock(_data_mutex);

  _point_cloud->data.clear();

  for (const auto& pt : m.point_cloud.data) {
    const std::array<float, 4> values = {
      static_cast<float>(pt.point.x()),
      static_cast<float>(pt.point.y()),
      static_cast<float>(pt.point.z()),
      static_cast<float>(pt.sigma)
    };
    const auto* bytes = reinterpret_cast<const uint8_t*>(values.data());
    _point_cloud->data.insert(_point_cloud->data.end(), bytes, bytes + _point_cloud->point_step);
  }

  _point_cloud->width    = static_cast<uint32_t>(m.nr_valid_points);
  _point_cloud->row_step = _point_cloud->point_step * _point_cloud->width;

  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    m.header.timestamp.time_since_epoch()).count();
  _point_cloud->header.stamp.sec     = static_cast<int32_t>(ns / 1'000'000'000LL);
  _point_cloud->header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);

  if (_callback_process_measurement) {
    _callback_process_measurement(*_point_cloud);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
