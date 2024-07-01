/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_point_cloud.hpp>

#include <memory>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

/**
 * \brief Fuses/Wraps multiple point cloud sensors to a single one.
 */
class SensorPointCloudFusion : public SensorPointCloud::SensorInterface
{
public:
  SensorPointCloudFusion(std::vector<std::shared_ptr<SensorPointCloud::SensorInterface>> sensor_hardware);

  void initialize(const SensorPointCloud::Parameter& parameter) override;

private:
  void processPointcloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index);

  std::vector<std::shared_ptr<SensorPointCloud::SensorInterface>> _sensor_hardware;
  std::vector<bool> _received_point_cloud;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> _point_cloud;
  std::mutex _data_mutex;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
