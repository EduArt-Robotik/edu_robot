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
 * \brief Adapter that subscribes to a single depth sensor of a MeasurementManager
 *        and publishes its measurements as a PointCloud2.
 */
class SensorTofHardwareAdapter : public SensorPointCloud::SensorInterface
{
public:
  SensorTofHardwareAdapter(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    unsigned int sensor_index);
  ~SensorTofHardwareAdapter() override = default;

  void initialize(const SensorPointCloud::Parameter& parameter) override;

private:
  void onDepthMeasurement(const sensorring::measurement::DepthMeasurement& m);

  std::shared_ptr<sensorring::manager::MeasurementManager> _manager;
  const unsigned int _sensor_index;

  std::unique_ptr<sensorring::subscription::Subscription> _depth_sub;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> _point_cloud;
  std::mutex _data_mutex;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
