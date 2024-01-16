/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cmath>

namespace eduart {
namespace robot {

class Rpm
{
public:
  static constexpr Rpm fromRps(const float value) { return Rpm(value * 60.0f); }
  static constexpr Rpm fromRadps(const float value) {
    return Rpm::fromRps(value / (2.0f * M_PI));
  }

  constexpr Rpm(const float rpm = 0.0f) : _value(rpm) { }

  inline constexpr float rps() const { return _value / 60.0f; }
  inline constexpr float radps() const { return rps() * 2.0f * M_PI; }
 
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
