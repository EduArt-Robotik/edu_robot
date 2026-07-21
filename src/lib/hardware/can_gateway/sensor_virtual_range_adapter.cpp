/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_virtual_range_adapter.hpp"

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

SensorVirtualRangeAdapter::SensorVirtualRangeAdapter(
  std::shared_ptr<sensorring::manager::MeasurementManager> manager,
  unsigned int sensor_index,
  const tf2::Transform& sensor_transform)
  : _manager(std::move(manager))
  , _sensor_index(sensor_index)
  , _virtual_range(std::make_unique<SensorVirtualRange>(sensor_transform))
  , _depth_sub(std::make_unique<sensorring::subscription::Subscription>())
{ }

void SensorVirtualRangeAdapter::initialize(const SensorRange::Parameter& parameter)
{
  _virtual_range->initialize(parameter);
  _virtual_range->registerCallbackProcessMeasurementData(
    [this](const float range) {
      if (_callback_process_measurement) {
        _callback_process_measurement(range);
      }
    }
  );

  auto depth_group = _manager->depthSensors();

  if (_sensor_index >= depth_group.size()) {
    throw std::out_of_range(
      "SensorVirtualRangeAdapter: sensor_index " + std::to_string(_sensor_index) +
      " is out of range (available: " + std::to_string(depth_group.size()) + ")"
    );
  }

  *_depth_sub = depth_group[_sensor_index].subscribe(
    std::bind(&SensorVirtualRangeAdapter::onDepthMeasurement, this, std::placeholders::_1)
  );
}

void SensorVirtualRangeAdapter::onDepthMeasurement(const sensorring::measurement::DepthMeasurement& m)
{
  // Convert the DepthMeasurement to the PointCloud2 format expected by SensorVirtualRange.
  // The virtual range filter operates in the sensor-local frame, so do NOT call
  // transformToGlobalFrame() here.
  sensor_msgs::msg::PointCloud2 point_cloud;
  point_cloud.height = 1;
  point_cloud.is_bigendian = false;
  point_cloud.point_step = 4u * sizeof(float); // x, y, z, sigma

  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;

  field.name = "x";     field.offset = 0u;  point_cloud.fields.push_back(field);
  field.name = "y";     field.offset = 4u;  point_cloud.fields.push_back(field);
  field.name = "z";     field.offset = 8u;  point_cloud.fields.push_back(field);
  field.name = "sigma"; field.offset = 12u; point_cloud.fields.push_back(field);

  point_cloud.data.reserve(m.point_cloud.data.size() * point_cloud.point_step);

  for (const auto& pt : m.point_cloud.data) {
    const std::array<float, 4> values = {
      static_cast<float>(pt.point.x()),
      static_cast<float>(pt.point.y()),
      static_cast<float>(pt.point.z()),
      static_cast<float>(pt.sigma)
    };
    const auto* bytes = reinterpret_cast<const uint8_t*>(values.data());
    point_cloud.data.insert(point_cloud.data.end(), bytes, bytes + point_cloud.point_step);
  }

  point_cloud.width    = static_cast<uint32_t>(m.point_cloud.data.size());
  point_cloud.row_step = point_cloud.point_step * point_cloud.width;

  _virtual_range->processPointCloudMeasurement(point_cloud);
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
