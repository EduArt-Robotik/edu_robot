/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_range.hpp>
#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"

#include <tf2/LinearMath/Transform.h>

#include <memory>

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
 * \brief Adapter that subscribes to a single depth sensor, converts the measurement
 *        to a PointCloud2, and feeds it into the existing SensorVirtualRange ground-plane
 *        filtering logic.
 *
 *        Replaces SensorVirtualRange wired through SensorTofHardware.
 */
class SensorVirtualRangeAdapter : public SensorRange::SensorInterface
{
public:
  SensorVirtualRangeAdapter(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    unsigned int sensor_index,
    const tf2::Transform& sensor_transform);
  ~SensorVirtualRangeAdapter() override = default;

  void initialize(const SensorRange::Parameter& parameter) override;

private:
  void onDepthMeasurement(const sensorring::measurement::DepthMeasurement& m);

  std::shared_ptr<sensorring::manager::MeasurementManager> _manager;
  const unsigned int _sensor_index;
  std::unique_ptr<SensorVirtualRange> _virtual_range;
  std::unique_ptr<sensorring::subscription::Subscription> _depth_sub;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
