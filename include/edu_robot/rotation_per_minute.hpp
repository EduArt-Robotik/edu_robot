/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace robot {

class Rpm
{
public:
  static constexpr Rpm fromRps(const float value) { return Rpm(value * 60.0f); }

  constexpr Rpm(const float rpm = 0.0f) : _value(rpm) { }

  inline constexpr float rps() const { return _value / 60.0f; }
 
  inline constexpr operator float() const { return _value; }
  constexpr Rpm& operator=(const float rhs)
  {
    _value = rhs;
    return *this;
  }

private:
  float _value = 0.0f;
};

} // end namespace eduart
} // end namespace robot
