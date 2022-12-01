/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/rotation_per_minute.hpp"
#include "edu_robot/angle.hpp"

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
class MotorController
{
public:
  struct Parameter
  {
    float gear_ratio = 89.0f;
    float encoder_ratio = 2048.0f;
    float max_rpm = 100.0f;
    float threshold_stall_check = 0.25f;
    std::uint32_t control_frequency = 16000;
    bool encoder_inverted = false;

    float kp = 0.5f;
    float ki = 5.0f;
    float kd = 0.0f;

    float weight_low_pass_set_point = 0.2f;
    float weight_low_pass_encoder   = 0.3f;

    bool isValid() const { return true; } // \todo implement properly
  };

  MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                  const std::string& urdf_joint_name, rclcpp::Node& ros_node,
                  std::shared_ptr<HardwareComponentInterface<Rpm>> hardware_component_interface,
                  std::shared_ptr<HardwareSensorInterface<Rpm>> hardware_sensor_interface);
  virtual ~MotorController();

  inline const std::string& name() const { return _name; }
  inline std::uint8_t id() const { return _id; }
  void setRpm(const Rpm rpm);
  inline Rpm getMeasuredRpm() const { return _measured_rpm; }

  static MotorController::Parameter get_motor_controller_parameter(
    const std::string& name, const MotorController::Parameter default_parameter, rclcpp::Node& ros_node);

private:
  void processMeasurementData(const Rpm measurement);

  Parameter _parameter;
  Rpm _set_rpm;
  Rpm _measured_rpm;
  std::string _name;
  std::uint8_t _id;
  std::string _urdf_joint_name;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> _pub_joint_state;
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _stamp_last_measurement;
  Angle0To2Pi _current_wheel_position = 0.0;

  std::shared_ptr<HardwareComponentInterface<Rpm>> _hardware_component_interface;
  std::shared_ptr<HardwareSensorInterface<Rpm>> _hardware_sensor_interface;
};

} // end namespace robot
} // end namespace eduart
