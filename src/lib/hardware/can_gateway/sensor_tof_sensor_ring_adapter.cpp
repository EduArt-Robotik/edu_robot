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
    std::shared_ptr<sensorring::manager::MeasurementManager> manager)
    : _manager(std::move(manager)),
      _point_cloud(make_point_cloud()) {}

void SensorTofSensorRingAdapter::initialize( const SensorPointCloud::Parameter &parameter) {
  (void)parameter;

  _depth_sub = std::make_unique<sensorring::subscription::Subscription>(_manager->depthSensors().subscribeAll(std::bind(&SensorTofSensorRingAdapter::onDepthMeasurement, this, std::placeholders::_1)));
}

SensorTofSensorRingAdapter::~SensorTofSensorRingAdapter() = default;

void SensorTofSensorRingAdapter::onDepthMeasurement( const std::vector<sensorring::measurement::DepthMeasurement> &m) {
  // Transform each valid point from sensor-local frame to ring-global frame
  for (const auto &measurement : m) {
    const auto global_cloud = measurement.transformToGlobalFrame();
    append_points_to_cloud(*_point_cloud, global_cloud.data);
  }

  finalize_cloud(*_point_cloud, rclcpp::Clock().now());

  if (_callback_process_measurement) {
    _callback_process_measurement(*_point_cloud);
    clear_point_cloud(*_point_cloud);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
