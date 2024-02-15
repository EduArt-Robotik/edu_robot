/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/sensor.hpp>
#include <edu_robot/processing_component/processing_component.hpp>
#include <edu_robot/diagnostic/standard_deviation.hpp>
#include <edu_robot/hardware_interface.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/range.hpp>

#include <memory>
#include <string>

namespace eduart {
namespace robot {

/**
 * \brief Represents a range sensor without a concrete hardware realization, that has to be implemented. This class
 *        creates an publisher for publishing its measurements. This class needs to be realized by a specific
 *        hardware layer.
 */
class SensorRange : public Sensor
                  , public processing::ProcessingComponentOutput<float>
{
public:
  struct Parameter {
    float field_of_view = 10.0f * M_PI / 180.0f;
    float range_min = 0.01f;
    float range_max = 5.0f;
  };

  struct SensorInterface : public HardwareInterface
                         , public HardwareComponent<Parameter>
                         , public HardwareSensor<const float> {
    SensorInterface() : HardwareInterface(HardwareInterface::Type::SENOR_IMU) { }
  };

  SensorRange(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
              const tf2::Transform sensor_transform, const Parameter parameter, rclcpp::Node& ros_node,
              std::shared_ptr<SensorInterface> hardware_interface);
  ~SensorRange() override = default;

protected:
  void processMeasurementData(const float measurement);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> _publisher;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;

  // diagnostic
  rclcpp::Time _last_processing;
  std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>> _processing_dt_statistic;  
};

} // end namespace eduart
} // end namespace robot
