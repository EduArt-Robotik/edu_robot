#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"
#include "edu_robot/algorithm/rotation.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <Eigen/Geometry>

#include <rclcpp/qos.hpp>

#include <limits>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

SensorVirtualRange::SensorVirtualRange(const tf2::Transform& sensor_transform)
  : _sensor_transform(sensor_transform)
{
  // remove translation so only rotation will be applied. --> point cloud will be axis aligned with the robot
  _sensor_transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
}

void SensorVirtualRange::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void SensorVirtualRange::processPointCloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud)
{
  const std::size_t number_of_points = point_cloud.data.size() / point_cloud.point_step;

  // Align given point cloud with robot's coordinate system (axis aligned)
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(_sensor_transform);
  algorithm::eliminate_yaw(transform.transform.rotation);

  sensor_msgs::msg::PointCloud2 point_cloud_transformed;
  tf2::doTransform(point_cloud, point_cloud_transformed, transform);

  if (number_of_points < 2) {
    // not enough points for processing
    return;
  }

  // find closest point
  std::vector<float> distances;
  
  distances.reserve(number_of_points);
  float distance = std::numeric_limits<float>::max();

  for (sensor_msgs::PointCloud2ConstIterator<float> point(point_cloud_transformed, "x"); point != point.end(); ++point) {
    // only process points that are above robot's ground plane
    if (point[2] < -_sensor_transform.getOrigin().getZ()) {
      continue;
    }

    distance = std::min(distance, point[0]); // \todo remove distance
    // take value or maximum numeric limit if distance is zero
    distances.push_back(point[0] == 0.0f ? std::numeric_limits<float>::max() : point[0]);
  }

  // sort distances and pick second closest one
  std::sort(distances.begin(), distances.end());

  if (_callback_process_measurement != nullptr) {
    // pick second closest one --> drop outlier
    _callback_process_measurement(distances[1]);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
