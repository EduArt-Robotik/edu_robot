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
  UART_SENDING_FAILED,
  UART_RECEIVING_FAILED,
  SHIELD_REQUEST_TIMEOUT,
  TCP_SOCKET_ERROR,
};

} // end namespace robot
} // end namespace eduart
