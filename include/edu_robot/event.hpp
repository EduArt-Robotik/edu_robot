/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace robot {

enum class Event {
  SHUTDOWN,
  EMERGENCY_STOP_PRESSED,
  EMERGENCY_STOP_RELEASED,
  BATTERY_EMPTY
};

} // end namespace robot
} // end namespace eduart