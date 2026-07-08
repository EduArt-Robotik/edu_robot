/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_point_cloud.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <mutex>
#include <vector>

// Forward declarations for sensorring types
namespace eduart {
namespace sensorring {
namespace manager { class MeasurementManager; }
namespace measurement { struct DepthMeasurement; }
namespace subscription { class Subscription; }
} // end namespace sensorring
} // end namespace eduart

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

/**
 * \brief Adapter that subscribes to all depth sensors of a MeasurementManager,
 *        accumulates one measurement cycle across all sensors, and publishes a merged PointCloud2.
 */
class SensorTofSensorRingAdapter : public SensorPointCloud::SensorInterface
{
public:
  SensorTofSensorRingAdapter(std::shared_ptr<sensorring::manager::MeasurementManager> manager);
  ~SensorTofSensorRingAdapter() override = default;

  void initialize(const SensorPointCloud::Parameter& parameter) override;

private:
  void onDepthMeasurement(const std::vector<sensorring::measurement::DepthMeasurement>& m);

  std::shared_ptr<sensorring::manager::MeasurementManager> _manager;

  // Stored as unique_ptr to avoid including sensorring/subscription/Subscription.hpp here
  std::unique_ptr<sensorring::subscription::Subscription> _depth_sub;

  // Fusion state (protected by _data_mutex, accessed from the manager worker thread)
  std::shared_ptr<sensor_msgs::msg::PointCloud2> _point_cloud;
  std::mutex _data_mutex;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
