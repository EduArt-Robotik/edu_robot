#include "edu_robot/bot/eduard_v3.hpp"
#include "edu_robot/hardware_error.hpp"

#include <edu_robot/sensor_point_cloud.hpp>

#include <algorithm>

namespace eduart {
namespace robot {
namespace bot {

static std::string get_port_name(const std::string& sensor_name)
{
  std::string result = sensor_name;
  std::replace(result.begin(), result.end(), '/', '.');
  return result;
}

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

  // Range Sensors
  constexpr std::array<const char*, 4> range_sensor_name = {
    "range/front/left", "range/front/right", "range/rear/left", "range/rear/right" };
  const std::array<tf2::Transform, 4> range_sensor_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17,  0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17, -0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17,  0.063, 0.050)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, -0.063, 0.050))
  };
  constexpr eduart::robot::SensorRange::Parameter range_sensor_parameter{ 10.0 * M_PI / 180.0, 0.01, 5.0 };

  for (std::size_t i = 0; i < range_sensor_name.size(); ++i) {
    auto range_sensor = std::make_shared<robot::SensorRange>(
      range_sensor_name[i],
      getFrameIdPrefix() + range_sensor_name[i],
      getFrameIdPrefix() + Robot::_parameter.tf.base_frame,
      range_sensor_pose[i],
      range_sensor_parameter,
      *this,
      factory.hardware().at(range_sensor_name[i])->cast<robot::SensorRange::SensorInterface>()
    );
    registerSensor(range_sensor);
    range_sensor->output("range")->connect(
      _collision_avoidance_component->input(get_port_name(range_sensor_name[i]))
    );
    factory.hardware().at(range_sensor_name[i])->cast<robot::SensorRange::SensorInterface>()->initialize(range_sensor_parameter);
  }

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

void EduardV3::setLightingForMode(const RobotMode mode)
{
  try {
    auto search_head = _lightings.find("head");
    auto search_back = _lightings.find("back");

    if (search_head == _lightings.end() || search_back == _lightings.end()) {
      RCLCPP_WARN(get_logger(), "Can't set lighting to indicate inactive mode. Lighting \"head\" or \"back\" was not found.");
      return;
    }

    switch (mode) {
      case RobotMode::REMOTE_CONTROLLED:
        search_head->second->setColor(Color{34, 34, 34}, Lighting::Mode::DIM);
        search_back->second->setColor(Color{34, 0, 0}, Lighting::Mode::DIM);
        break;

      default:
        // For all other modes use the default implementation of the base class.
        Robot::setLightingForMode(mode);
        break;
    }
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to set new values for lighting \"all\"."
                                      << " what() = " << ex.what());                                      
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to set new values for lighting \"all\"."
                                      << " what() = " << ex.what());     
  }
}

} // end namespace bot
} // end namespace robot
} // end namespace eduart
