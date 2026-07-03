/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_tof_sensor_ring_adapter.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_point_cloud_utils.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/measurement/DepthMeasurement.hpp>
#include <sensorring/subscription/Subscription.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

SensorTofSensorRingAdapter::SensorTofSensorRingAdapter(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    std::size_t expected_sensor_count)
    : _manager(std::move(manager)),
      _expected_sensor_count(expected_sensor_count),
      _depth_sub(std::make_unique<sensorring::subscription::Subscription>()),
      _received(expected_sensor_count, false),
      _point_cloud(make_point_cloud()) {}

void SensorTofSensorRingAdapter::initialize(
    const SensorPointCloud::Parameter &parameter) {
  (void)parameter;

  *_depth_sub = _manager->depthSensors().subscribe(
      std::bind(&SensorTofSensorRingAdapter::onDepthMeasurement, this,
                std::placeholders::_1));

  if (!_manager->isMeasuring()) {
    _manager->startMeasuring();
  }
}

void SensorTofSensorRingAdapter::onDepthMeasurement(
    const sensorring::measurement::DepthMeasurement &m) {
  const auto idx = m.header.device_id.index;

  if (idx >= _expected_sensor_count) {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("SensorTofSensorRingAdapter"),
                     "received measurement for out-of-range sensor index %u "
                     "(expected < %zu); ignoring",
                     idx, _expected_sensor_count);
    return;
  }

  std::scoped_lock lock(_data_mutex);

  if (_received[idx]) {
    // Late arrival from a previous cycle — skip to avoid mixing cycles
    return;
  }

  // Transform each valid point from sensor-local frame to ring-global frame
  const auto global_cloud = m.transformToGlobalFrame();
  append_points_to_cloud(*_point_cloud, global_cloud.data);

  _received[idx] = true;

  // Fire callback once all sensors have delivered for this cycle
  const bool all_received =
      std::all_of(_received.begin(), _received.end(), [](bool v) { return v; });

  if (all_received) {
    finalize_cloud(*_point_cloud, rclcpp::Clock().now());

    if (_callback_process_measurement) {
      _callback_process_measurement(*_point_cloud);
    }

    clearProcessing();
  }
}

void SensorTofSensorRingAdapter::clearProcessing() {
  std::fill(_received.begin(), _received.end(), false);
  clear_point_cloud(*_point_cloud);
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
