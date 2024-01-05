/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/rpm.hpp"
#include "edu_robot/angle.hpp"

#include <cstddef>
#include <edu_robot/action/motor_action.hpp>

#include <edu_robot/diagnostic/diagnostic_component.hpp>
#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdint>
#include <string>
#include <memory>

namespace eduart {
namespace robot {

/**
 * \brief Represents a hardware motor controller, but without concrete realization.
 *        This class needs to be realized by a specific hardware layer.
 */
class MotorController : public diagnostic::DiagnosticComponent
{
public:
  friend class action::CheckIfMotorIsEnabled;

  struct Parameter
  {
    bool inverted = false;
    float gear_ratio = 89.0f;
    float encoder_ratio = 2048.0f;
    float max_rpm = 100.0f;
    float threshold_stall_check = 0.25f;
    std::uint32_t control_frequency = 16000;
    bool encoder_inverted = false;
    bool closed_loop = true;
    std::size_t index = 0;
    std::uint32_t timeout_ms = 1000;

    float kp = 0.5f;
    float ki = 5.0f;
    float kd = 0.0f;

    float weight_low_pass_set_point = 0.2f;
    float weight_low_pass_encoder   = 0.3f;

    bool isValid() const { return true; } // \todo implement properly
  };

  using ComponentInterface = HardwareComponentInterface<Parameter, Rpm>;
  using SensorInterface = HardwareSensorInterface<Parameter, Rpm, bool>;

  MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                  const std::string& urdf_joint_name, rclcpp::Node& ros_node,
                  std::shared_ptr<ComponentInterface> hardware_component_interface,
                  std::shared_ptr<SensorInterface> hardware_sensor_interface);
  virtual ~MotorController();

  inline const std::string& name() const { return _name; }
  inline std::uint8_t id() const { return _id; }
  /**
   * \brief Sets RPM of this motor. Positive RPM 
   */
  void setRpm(const Rpm rpm);
  inline Rpm getMeasuredRpm() const {
    std::lock_guard guard(_mutex_access_data);
    return _measured_rpm;
  }
  inline bool isEnabled() const {
    std::lock_guard guard(_mutex_access_data);
    return _enabled;    
  }

  static MotorController::Parameter get_parameter(
    const std::string& name, const MotorController::Parameter& default_parameter, rclcpp::Node& ros_node);
  const Parameter& parameter() const { return _parameter; }

private:
  void processMeasurementData(const Rpm measurement, const bool enabled_flag);
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  Rpm _set_rpm;
  Rpm _measured_rpm;
  bool _enabled = false;
  std::string _name;
  std::uint8_t _id;
  std::string _urdf_joint_name;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> _pub_joint_state;
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _stamp_last_measurement;
  Angle0To2Pi _current_wheel_position = 0.0;
  mutable std::mutex _mutex_access_data;

  std::shared_ptr<ComponentInterface> _hardware_component_interface;
  std::shared_ptr<SensorInterface> _hardware_sensor_interface;

  // diagnostic
  rclcpp::Time _last_processing;
  std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>> _processing_dt_statistic;
  std::atomic_bool _lost_enable = false;
};

} // end namespace robot
} // end namespace eduart
