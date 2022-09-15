/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace robot {

enum class State {
  OK,
  CHARGER_CONNECTED,
  MOTOR_ERROR,
  LOW_BATTERY,
  EMERGENCY_STOP,
};

} // end namespace robot
} // end namespace eduart
