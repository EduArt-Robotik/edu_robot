/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_range.hpp>
#include <edu_robot/plane.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class SensorVirtualRange : public SensorRange::SensorInterface
{
public:
  SensorVirtualRange(const tf2::Transform& sensor_transform);
  ~SensorVirtualRange() override = default;

  void initialize(const SensorRange::Parameter& parameter) override;

  void processPointCloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud);

private:
  tf2::Transform _sensor_transform;
  Plane<double> _ground_plane;
};               

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
