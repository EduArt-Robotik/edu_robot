/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstdint>

namespace eduart {
namespace robot {

struct Color
{
  static constexpr Color fromRgb(const std::uint8_t r, const std::uint8_t g, const std::uint8_t b) { return {r, g, b}; }
  // \todo add method fromHsv, usw...

  Color operator*(const float &factor) const {
    return {static_cast<std::uint8_t>(r * factor),
            static_cast<std::uint8_t>(g * factor),
            static_cast<std::uint8_t>(b * factor)};
  }

  Color &operator*=(const float &factor) {
    *this = *this * factor;
    return *this;
  }

  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
};

} // end namespace eduart
} // end namespace robot
