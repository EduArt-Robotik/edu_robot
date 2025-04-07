#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"
#include <edu_robot/algorithm/rotation.hpp>
#include <edu_robot/algorithm/geometry.hpp>
#include <edu_robot/message/geometry.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
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
  // const Eigen::Quaterniond camera_to_robot(
  //   Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
  // const std::size_t number_of_points = point_cloud.data.size() / point_cloud.point_step;

  // // Extract rotation into robot like coordinate system (x-direction in front)
  // const Eigen::Quaterniond to_robot_frame = message::from_ros(tf2::toMsg(_sensor_transform).rotation);
  // Eigen::Quaterniond sensor_to_robot = to_robot_frame * camera_to_robot.inverse();

  // // Eliminate yaw rotation
  // algorithm::eliminate_yaw(sensor_to_robot);

  // // Align given point cloud with robot's coordinate system (axis aligned)
  // geometry_msgs::msg::TransformStamped transform;
  // // transform.transform.rotation = message::to_ros(sensor_to_robot * camera_to_robot);
  // transform.transform.rotation = tf2::toMsg(_sensor_transform.getRotation());

  // sensor_msgs::msg::PointCloud2 point_cloud_transformed;
  // tf2::doTransform(point_cloud, point_cloud_transformed, transform);

  // if (number_of_points < 2) {
  //   // not enough points for processing
  //   return;
  // }

  // // find closest point
  // std::vector<float> distances;
  
  // distances.reserve(number_of_points);
  // float distance = std::numeric_limits<float>::max();

  // for (sensor_msgs::PointCloud2ConstIterator<float> point(point_cloud_transformed, "x"); point != point.end(); ++point) {
  //   std::cout << "point: " << point[0] << ", " << point[1] << ", " << point[2] << std::endl;

  //   // only process points that are above robot's ground plane
  //   if (std::isnan(point[0]) || point[0] == 0.0f || point[2] < -_sensor_transform.getOrigin().getZ()) {
  //     distances.push_back(std::numeric_limits<float>::max());
  //     continue;
  //   }
  //   // else: valid point

  //   distance = std::min(distance, point[0]); // \todo remove distance
  //   // take value or maximum numeric limit if distance is zero
  //   distances.push_back(std::abs(point[0]));
  // }

  // // sort distances and pick second closest one
  // std::sort(distances.begin(), distances.end());

  // for (const auto& distance : distances) {
  //   std::cout << distance << ",";
  // }
  // std::cout << std::endl;

  // if (_callback_process_measurement != nullptr) {
  //   // pick second closest one --> drop outlier
  //   std::cout << "distance = " << distances[1] << std::endl;
  //   _callback_process_measurement(distances[1]);
  // }

  // alternative implementation
  // define a plane and transform it into sensor coordinate system
  // the plane represents the robots ground plane
  Plane<double> ground_plane(Eigen::Vector3d(0, 0, -_sensor_transform.getOrigin().z()), Eigen::Vector3d::UnitZ());
  // only rotate, because plane already translated at construction
  algorithm::rotate_plane(ground_plane, message::from_ros(_sensor_transform.getRotation()));

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
    if (std::isnan(p.z()) || p.z() == 0.0f || algorithm::is_above_plane(ground_plane, p) == false) {
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
    std::cout << "distance = " << distances[1] << std::endl;
    _callback_process_measurement(distances[1]);
  }  
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
