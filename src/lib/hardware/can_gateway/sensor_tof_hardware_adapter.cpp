/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware_adapter.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_point_cloud_utils.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/measurement/DepthMeasurement.hpp>
#include <sensorring/subscription/Subscription.hpp>

#include <rclcpp/clock.hpp>

#include <functional>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

SensorTofHardwareAdapter::SensorTofHardwareAdapter(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    unsigned int sensor_index)
    : _manager(std::move(manager)), _sensor_index(sensor_index),
      _depth_sub(std::make_unique<sensorring::subscription::Subscription>()),
      _point_cloud(make_point_cloud()) {}

void SensorTofHardwareAdapter::initialize(
    const SensorPointCloud::Parameter &parameter) {
  (void)parameter;

  auto depth_group = _manager->depthSensors();

  if (_sensor_index >= depth_group.size()) {
    throw std::out_of_range("SensorTofHardwareAdapter: sensor_index " +
                            std::to_string(_sensor_index) +
                            " is out of range (available: " +
                            std::to_string(depth_group.size()) + ")");
  }

  *_depth_sub = depth_group[_sensor_index].subscribe(
      std::bind(&SensorTofHardwareAdapter::onDepthMeasurement, this,
                std::placeholders::_1));
}

void SensorTofHardwareAdapter::onDepthMeasurement(
    const sensorring::measurement::DepthMeasurement &m) {
  std::scoped_lock lock(_data_mutex);

  clear_point_cloud(*_point_cloud);
  append_points_to_cloud(*_point_cloud, m.point_cloud.data);
  finalize_cloud(*_point_cloud, rclcpp::Clock().now());

  if (_callback_process_measurement) {
    _callback_process_measurement(*_point_cloud);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
