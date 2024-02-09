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

#include <tf2/tf2/LinearMath/Transform.h>

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
    struct {
      std::size_t vertical = 8;
      std::size_t horizontal = 8;
    } number_of_zones; // number of sensor zones (pixel)
    struct {
      Angle vertical = Angle::createFromDegree(45);
      Angle horizontal = Angle::createFromDegree(45);
    } fov;
    std::chrono::milliseconds measurement_interval;
  };

  struct SensorInterface : public HardwareInterface
                         , public HardwareComponent<Parameter>
                         , public HardwareSensor<std::size_t, float, float> {
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
  void processMeasurementData(const std::size_t zone_index, const float distance, const float sigma);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> _point_cloud;
  struct {
    std::size_t number_of_zones;
    std::size_t current_zone;
    std::size_t next_expected_zone;
    std::vector<float> tan_x_lookup; // used to transform to point y
    std::vector<float> tan_y_lookup; // used to transform to point x
  } _processing_data;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> _publisher;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;                        
};

} // end namespace eduart
} // end namespace robot
