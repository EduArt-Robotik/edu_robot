#pragma once

namespace eduart_robot {

enum class Status {
  OK,
  CHARGER_CONNECTED,
  MOTOR_ERROR,
  LOW_BATTERY,
  EMERGENCY_STOP,
};

} // end namespace eduart_robot