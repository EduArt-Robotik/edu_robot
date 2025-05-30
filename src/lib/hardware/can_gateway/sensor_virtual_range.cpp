#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"
#include <edu_robot/algorithm/rotation.hpp>
#include <edu_robot/algorithm/geometry.hpp>
#include <edu_robot/message/geometry.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Vector3.h>

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
  , _ground_plane(Eigen::Vector3d(0, 0, -_sensor_transform.getOrigin().z()), Eigen::Vector3d::UnitZ())
{
  // define a plane and transform it into sensor coordinate system
  // the plane represents the robots ground plane
  // only rotate, because plane already translated at construction
  algorithm::rotate_plane(_ground_plane, message::from_ros(_sensor_transform.getRotation()).conjugate());
}

void SensorVirtualRange::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void SensorVirtualRange::processPointCloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud)
{
  // process points
  const std::size_t number_of_points = point_cloud.data.size() / point_cloud.point_step;  
  std::vector<float> distances;
  distances.reserve(number_of_points);

  if (number_of_points < 2) {
    // not enough points for processing
    _callback_process_measurement(std::numeric_limits<float>::max());
    return;
  }

  // find closest point
  for (sensor_msgs::PointCloud2ConstIterator<float> point(point_cloud, "x"); point != point.end(); ++point) {  
    const Eigen::Vector3d p(point[0], point[1], point[2]);

    // only process points that are above robot's ground plane and valid
    if (std::isnan(p.z()) || p.z() == 0.0f || algorithm::is_above_plane(_ground_plane, p) == false) {
      distances.push_back(std::numeric_limits<float>::max());
      continue;      
    }
    //else: valid distance measurement
    distances.push_back(p.z());
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
