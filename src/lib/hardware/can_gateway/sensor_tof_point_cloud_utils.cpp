/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/sensor_tof_point_cloud_utils.hpp"

#include <sensor_msgs/msg/point_field.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

std::shared_ptr<sensor_msgs::msg::PointCloud2> make_point_cloud() {
  auto point_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  point_cloud->height = 1;
  point_cloud->width = 0;
  point_cloud->is_bigendian = false;
  point_cloud->point_step = 5u * sizeof(float) + sizeof(uint8_t); // x, y, z, sigma, raw_distance, sensor_idx
  point_cloud->row_step = 0u;

  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;

  field.name = "x";
  field.offset = 0u;
  point_cloud->fields.push_back(field);
  field.name = "y";
  field.offset = 4u;
  point_cloud->fields.push_back(field);
  field.name = "z";
  field.offset = 8u;
  point_cloud->fields.push_back(field);
  field.name = "sigma";
  field.offset = 12u;
  point_cloud->fields.push_back(field);
  field.name = "raw_distance";
  field.offset = 16u;
  point_cloud->fields.push_back(field);
  field.name = "sensor_idx";
  field.offset = 20u;
  field.datatype = sensor_msgs::msg::PointField::UINT8;
  point_cloud->fields.push_back(field);

  return point_cloud;
}

void append_point_to_cloud(sensor_msgs::msg::PointCloud2 &point_cloud, float x, float y, float z, float sigma, float raw_distance, std::uint8_t sensor_idx) {
  const std::array<float, 5> values = {x, y, z, sigma, raw_distance};
  const auto *bytes = reinterpret_cast<const uint8_t *>(values.data());
  point_cloud.data.insert(point_cloud.data.end(), bytes, bytes + 5 * sizeof(float));
  point_cloud.data.push_back(sensor_idx);
}

void finalize_cloud(sensor_msgs::msg::PointCloud2 &point_cloud, const rclcpp::Time &stamp) {
  const auto width = static_cast<uint32_t>(point_cloud.data.size() / point_cloud.point_step);
  point_cloud.header.stamp = stamp;
  point_cloud.width = width;
  point_cloud.row_step = point_cloud.point_step * point_cloud.width;
}

void clear_point_cloud(sensor_msgs::msg::PointCloud2 &point_cloud) {
  point_cloud.data.clear();
  point_cloud.width = 0u;
  point_cloud.row_step = 0u;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
