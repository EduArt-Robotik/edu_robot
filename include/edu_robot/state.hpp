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
  UART_HARDWARE_ERROR,
  SHIELD_REQUEST_TIMEOUT,
  UDP_SOCKET_ERROR,
  CAN_SOCKET_ERROR,
  FUNCTIONAL_ERROR,
};

} // end namespace robot
} // end namespace eduart
