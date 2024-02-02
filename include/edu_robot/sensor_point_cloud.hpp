/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interfaces.hpp"
#include "edu_robot/sensor.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/tf2/LinearMath/Transform.h>

namespace eduart {
namespace robot {

/**
 * \brief Represents a single or a collection of EduArt point cloud sensors. These sensors are usually mounted around
 *        a robot. This class needs to be realized by a specific hardware layer.
 */
class PointCloudSensor : public Sensor
{
public:
  struct Parameter {
    std::size_t number_of_zones;
  };

  struct SensorInterface : public HardwareComponent<Parameter>
                         , public HardwareSensor<std::size_t, float, float> { };

  PointCloudSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
              const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
              std::shared_ptr<SensorInterface> hardware_interface);
  ~PointCloudSensor() override = default;

protected:
  void processMeasurementData(const std::size_t zone_index, const float distance, const float sigma);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> _point_cloud;
  std::size_t _current_zone;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _publisher;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;                        
};

} // end namespace eduart
} // end namespace robot
