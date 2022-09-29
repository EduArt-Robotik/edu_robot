/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

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
    float gear_ratio = 70.0f;
    float encoder_ratio = 2048.0f;
    float max_rpm = 140.0f;
    std::uint32_t control_frequency = 16000;

    float ki = 5.0f;
    float kd = 0.0f;
    float kp = 0.5f;

    float weight_low_pass_set_point = 0.2f;
    float weight_low_pass_encoder   = 0.3f;

    bool isValid() const { return true; } // \todo implement properly
  };

protected:
  MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                  const std::string& urdf_joint_name, rclcpp::Node& ros_node);

public:
  virtual ~MotorController();

  inline const std::string& name() const { return _name; }
  inline std::uint8_t id() const { return _id; }
  virtual void initialize(const Parameter& parameter) = 0;
  void setRpm(const Rpm rpm);
  inline Rpm getMeasuredRpm() const { return _measured_rpm; }

protected:
  virtual void processSetRpm(const Rpm rpm) = 0;
  void processMeasurementData(const Rpm measurement);

  Parameter _parameter;
  Rpm _set_rpm;

private:
  Rpm _measured_rpm;
  std::string _name;
  std::uint8_t _id;
  std::string _urdf_joint_name;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> _pub_joint_state;
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _stamp_last_measurement;
  Angle0To2Pi _current_wheel_position = 0.0;
};

} // end namespace robot
} // end namespace eduart
