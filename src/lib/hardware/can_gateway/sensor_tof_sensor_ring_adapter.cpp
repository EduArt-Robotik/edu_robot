/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_tof_sensor_ring_adapter.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/measurement/DepthMeasurement.hpp>
#include <sensorring/subscription/Subscription.hpp>

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cstring>
#include <functional>

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

SensorTofSensorRingAdapter::SensorTofSensorRingAdapter(
  std::shared_ptr<sensorring::manager::MeasurementManager> manager,
  std::size_t expected_sensor_count)
  : _manager(std::move(manager))
  , _expected_sensor_count(expected_sensor_count)
  , _depth_sub(std::make_unique<sensorring::subscription::Subscription>())
  , _received(expected_sensor_count, false)
  , _point_cloud(create_point_cloud())
{ }

void SensorTofSensorRingAdapter::initialize(const SensorPointCloud::Parameter& parameter)
{
  (void)parameter;

  *_depth_sub = _manager->depthSensors().subscribe(
    std::bind(&SensorTofSensorRingAdapter::onDepthMeasurement, this, std::placeholders::_1)
  );

  if (!_manager->isMeasuring()) {
    _manager->startMeasuring();
  }
}

void SensorTofSensorRingAdapter::onDepthMeasurement(const sensorring::measurement::DepthMeasurement& m)
{
  const auto idx = m.header.device_id.index;

  if (idx >= _expected_sensor_count) {
    RCLCPP_WARN_ONCE(
      rclcpp::get_logger("SensorTofSensorRingAdapter"),
      "received measurement for out-of-range sensor index %u (expected < %zu); ignoring",
      idx, _expected_sensor_count);
    return;
  }

  std::scoped_lock lock(_data_mutex);

  if (_received[idx]) {
    // Late arrival from a previous cycle — skip to avoid mixing cycles
    return;
  }

  // Transform each valid point from sensor-local frame to ring-global frame
  // (copy required because transformToGlobalFrame() is non-const)
  sensorring::measurement::DepthMeasurement mutable_m = m;
  const auto global_cloud = mutable_m.transformToGlobalFrame();

  for (const auto& pt : global_cloud.data) {
    const std::array<float, 4> values = {
      static_cast<float>(pt.point.x()),
      static_cast<float>(pt.point.y()),
      static_cast<float>(pt.point.z()),
      static_cast<float>(pt.sigma)
    };
    const auto* bytes = reinterpret_cast<const uint8_t*>(values.data());
    _point_cloud->data.insert(_point_cloud->data.end(), bytes, bytes + _point_cloud->point_step);
  }

  _received[idx] = true;

  // Fire callback once all sensors have delivered for this cycle
  const bool all_received = std::all_of(_received.begin(), _received.end(), [](bool v) { return v; });

  if (all_received) {
    _point_cloud->width     = static_cast<uint32_t>(_point_cloud->data.size() / _point_cloud->point_step);
    _point_cloud->row_step  = _point_cloud->point_step * _point_cloud->width;

    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      m.header.timestamp.time_since_epoch()).count();
    _point_cloud->header.stamp.sec     = static_cast<int32_t>(ns / 1'000'000'000LL);
    _point_cloud->header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);

    if (_callback_process_measurement) {
      _callback_process_measurement(*_point_cloud);
    }

    clearProcessing();
  }
}

void SensorTofSensorRingAdapter::clearProcessing()
{
  std::fill(_received.begin(), _received.end(), false);
  _point_cloud->data.clear();
  _point_cloud->width    = 0u;
  _point_cloud->row_step = 0u;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
