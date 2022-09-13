#pragma once

namespace eduart {
namespace robot {

enum class Status {
  OK,
  CHARGER_CONNECTED,
  MOTOR_ERROR,
  LOW_BATTERY,
  EMERGENCY_STOP,
};

} // end namespace robot
} // end namespace eduart
