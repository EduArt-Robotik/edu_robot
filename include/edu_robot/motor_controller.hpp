/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rotation_per_minute.hpp"

#include <cstdint>
#include <string>

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
    float encoder_ratio = 64.0f;
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
  MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter);

public:
  virtual ~MotorController();

  inline const std::string& name() const { return _name; }
  inline std::uint8_t id() const { return _id; }
  virtual void initialize(const Parameter& parameter) = 0;
  void setRpm(const Rpm rpm);
  inline Rpm getMeasuredRpm() const { return _measured_rpm; }

protected:
  virtual void processSetRpm(const Rpm rpm) = 0;

  Parameter _parameter;
  Rpm _set_rpm;
  Rpm _measured_rpm;

private:
  std::string _name;
  std::uint8_t _id;
};

} // end namespace robot
} // end namespace eduart
