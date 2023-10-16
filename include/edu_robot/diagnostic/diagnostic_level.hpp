/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace eduart {
namespace robot {
namespace diagnostic {

enum class Level {
  OK = 1,
  WARN,
  ERROR, // needs to be the highest value
};

inline std::remove_const_t<decltype(diagnostic_msgs::msg::DiagnosticStatus::OK)> convert(
  const Level level)
{
  switch (level) {
    case Level::OK:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;

    case Level::WARN:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;

    case Level::ERROR:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    default:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
}

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
