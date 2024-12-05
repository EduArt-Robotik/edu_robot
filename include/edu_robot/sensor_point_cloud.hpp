/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/sensor.hpp>
#include <edu_robot/angle.hpp>
#include <edu_robot/hardware_interface.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Transform.h>

namespace eduart {
namespace robot {

/**
 * \brief Represents a single or a collection of EduArt point cloud sensors. These sensors are usually mounted around
 *        a robot. This class needs to be realized by a specific hardware layer.
 */
class SensorPointCloud : public Sensor
{
public:
  struct Parameter {

  };

  struct SensorInterface : public HardwareInterface
                         , public HardwareComponent<Parameter>
                         , public HardwareSensor<sensor_msgs::msg::PointCloud2> {
    SensorInterface() : HardwareInterface(HardwareInterface::Type::SENSOR_POINT_CLOUD) { }
  };

  SensorPointCloud(
    const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
    const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
    std::shared_ptr<SensorInterface> hardware_interface);
  ~SensorPointCloud() override = default;

static Parameter get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

protected:
  void processMeasurementData(sensor_msgs::msg::PointCloud2& point_cloud);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _publisher;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;                        
};

} // end namespace eduart
} // end namespace robot
