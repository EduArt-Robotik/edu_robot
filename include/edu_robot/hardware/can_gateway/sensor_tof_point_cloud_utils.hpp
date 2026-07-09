/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Hannes Duske (hannes.duske@eduart-robotik.com)
 */
#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/time.hpp>

#include <array>
#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

std::shared_ptr<sensor_msgs::msg::PointCloud2> make_point_cloud();

void append_point_to_cloud(sensor_msgs::msg::PointCloud2 &point_cloud, float x, float y, float z, float sigma, float raw_distance, std::uint8_t sensor_idx);

template <typename PointRange>
void append_points_to_cloud(sensor_msgs::msg::PointCloud2 &point_cloud, const PointRange &points) {
  for (const auto &pt : points) {
    append_point_to_cloud(point_cloud, static_cast<float>(pt.point.x()),
                          static_cast<float>(pt.point.y()),
                          static_cast<float>(pt.point.z()),
                          static_cast<float>(pt.sigma),
                          static_cast<float>(pt.raw_distance),
                          static_cast<std::uint8_t>(pt.sensor_index));
  }
}

void finalize_cloud(sensor_msgs::msg::PointCloud2 &point_cloud, const rclcpp::Time &stamp);

void clear_point_cloud(sensor_msgs::msg::PointCloud2 &point_cloud);

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
