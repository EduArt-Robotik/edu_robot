#include "edu_robot/bot/eduard_v3.hpp"

#include <edu_robot/sensor_point_cloud.hpp>

namespace eduart {
namespace robot {
namespace bot {

EduardV3::EduardV3(
  const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns)
  : Eduard(robot_name, std::move(hardware_interface), ns)
{

}

EduardV3::~EduardV3()
{

}

void EduardV3::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // First Calling initialize of Base Class
  Eduard::initialize(factory);

  // Point Cloud Sensors
  constexpr std::array<const char*, 1> point_cloud_name = { "pointcloud" };
  constexpr std::array<const char*, 1> point_cloud_tf = { "pointcloud" };
  const std::array<tf2::Transform, 1> point_cloud_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, 0.0))
  };
  
  for (std::size_t i = 0; i < point_cloud_name.size(); ++i) {
    const auto parameter = robot::SensorPointCloud::get_parameter(
      point_cloud_name[i], {}, *this);
    auto hardware_interface = factory.hardware().at("tof_sensor_ring")->cast<robot::SensorPointCloud::SensorInterface>();
    auto point_cloud_sensor = std::make_shared<robot::SensorPointCloud>(
      point_cloud_name[i],
      getFrameIdPrefix() + point_cloud_tf[i],
      getFrameIdPrefix() + Robot::_parameter.tf.base_frame,
      point_cloud_pose[i],
      parameter,
      *this,
      hardware_interface
    );
    registerSensor(point_cloud_sensor);
    hardware_interface->initialize(parameter);
  }  
}

} // end namespace bot
} // end namespace robot
} // end namespace eduart
