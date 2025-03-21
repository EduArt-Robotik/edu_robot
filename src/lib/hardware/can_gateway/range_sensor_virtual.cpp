#include "edu_robot/hardware/can_gateway/range_sensor_virtual.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <memory>
#include <rclcpp/qos.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

RangeSensorVirtual::RangeSensorVirtual(const tf2::Transform& sensor_transform)
  : _sensor_transform(sensor_transform)
{
  
}

void RangeSensorVirtual::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void RangeSensorVirtual::processPointCloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud)
{

}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
