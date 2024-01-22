/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rpm.hpp"
#include "edu_robot/angle.hpp"

#include <edu_robot/action/motor_action.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/node.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <cstdint>

namespace eduart {
namespace robot {

class Motor
{
public:
  /**
   * \brief Motor Parameter
   */
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

  Motor(const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, const std::string& urdf_joint_name);

  inline const std::string& name() const { return _name; }
  void processMeasurementData(const Rpm rpm, const bool enabled_flag, const rclcpp::Time& stamp);
  inline bool isEnabled() const { return _enabled; }
  static Parameter get_parameter(
    const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);
  const Parameter& parameter() const { return _parameter; }

private:
  const Parameter _parameter;
  Rpm _set_rpm;
  Rpm _measured_rpm;
  bool _enabled = false;
  std::string _name;
  std::uint8_t _id;
  std::string _urdf_joint_name;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> _pub_joint_state;
  Angle0To2Pi _current_wheel_position = 0.0;
  rclcpp::Time _stamp_last_measurement;   
};

} // end namespace robot
} // end namespace eduart
