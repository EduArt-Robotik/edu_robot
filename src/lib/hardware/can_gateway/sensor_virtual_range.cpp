#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <limits>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Vector3.hpp>

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
  // Align given point cloud with robot's coordinate system (axis aligned)
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(_sensor_transform);
  sensor_msgs::msg::PointCloud2 point_cloud_transformed;
  tf2::doTransform(point_cloud, point_cloud_transformed, transform);

  // find closest point
  float distance = std::numeric_limits<float>::max();

  for (sensor_msgs::PointCloud2ConstIterator<float> point(point_cloud_transformed, "x"); point != point.end(); ++point) {
    distance = std::min(distance, point[0]);
  }

  if (_callback_process_measurement != nullptr) {
    _callback_process_measurement(distance);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
