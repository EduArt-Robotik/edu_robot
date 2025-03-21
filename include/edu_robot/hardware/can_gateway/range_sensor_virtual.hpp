/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_range.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class RangeSensorVirtual : public SensorRange::SensorInterface
{
public:
  RangeSensorVirtual(const tf2::Transform& sensor_transform);
  ~RangeSensorVirtual() override = default;

  void initialize(const SensorRange::Parameter& parameter) override;

  void processPointCloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud);

private:
  tf2::Transform _sensor_transform;
};               

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
