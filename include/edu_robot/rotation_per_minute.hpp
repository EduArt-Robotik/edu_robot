/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <stdexcept>

namespace eduart {
namespace robot {

class Rpm
{
public:
  static constexpr Rpm fromRps(const float value) { return Rpm(value * 60.0f); }

  constexpr Rpm(const float rpm) { setRpm(rpm); }

  inline constexpr float rps() const { return _value / 60.0f; }
 
  inline constexpr operator float() const { return _value; }
  constexpr Rpm& operator=(const float rhs)
  {
    setRpm(rhs);
    return *this;
  }

private:
  constexpr void setRpm(const float value)
  {
    if (value < 0.0) {
      throw std::invalid_argument("given rpm value is negative");
    }

    _value = value;
  }

  float _value = 0.0f;
};

} // end namespace eduart
} // end namespace robot