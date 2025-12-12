/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/message_buffer.hpp>

#include <cstdint>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
namespace can {
namespace message {

using hardware::message::Byte;

struct PROTOCOL {
  struct MOTOR {
    struct COMMAND {
      static constexpr Byte ENABLE = 0x01;
      static constexpr Byte DISABLE = 0x02;
      static constexpr Byte SET_TIMEOUT = 0x03;
      static constexpr Byte INVERT_ENCODER = 0x07;
      static constexpr Byte SET_PWM = 0x10;
      static constexpr Byte SET_RPM = 0x11;
      static constexpr Byte OPEN_LOOP = 0x09;
      static constexpr Byte CLOSE_LOOP = 0x08;
      static constexpr Byte FREQUENCY = 0x12;
      static constexpr Byte CTL_KP = 0x20;
      static constexpr Byte CTL_KI = 0x21;
      static constexpr Byte CTL_KD = 0x22;
      static constexpr Byte CTL_ANTI_WIND_UP = 0x23;
      static constexpr Byte CTL_INPUT_FILTER = 0x24;
      static constexpr Byte GEAR_RATIO = 0x30;
      static constexpr Byte GEAR_RATIO2 = 0x31;
      static constexpr Byte TICKS_PER_REVISION = 0x32;
      static constexpr Byte TICKS_PER_REVISION2 = 0x33;
      static constexpr Byte SET_RPM_MAX = 0x04;

      static constexpr Byte GET_FIRMWARE = 0x0a;
      static constexpr Byte GET_TIMEOUT = SET_TIMEOUT | 0x80;
      static constexpr Byte GET_RPM_MAX = SET_RPM_MAX | 0x80;
      static constexpr Byte GET_INVERTED_ENCODER = INVERT_ENCODER | 0x80;
      static constexpr Byte GET_CLOSED_LOOP = CLOSE_LOOP | 0x80;
      static constexpr Byte GET_MOTOR_GEAR_RATIO = GEAR_RATIO | 0x80;
      static constexpr Byte GET_MOTOR_GEAR_RATIO2 = GEAR_RATIO2 | 0x80;
      static constexpr Byte GET_TICKS_PER_REVISION = TICKS_PER_REVISION | 0x80;
      static constexpr Byte GET_TICKS_PER_REVISION2 = TICKS_PER_REVISION2 | 0x80;
      static constexpr Byte GET_CTL_KP = CTL_KP | 0x80;
      static constexpr Byte GET_CTL_KI = CTL_KI | 0x80;
      static constexpr Byte GET_CTL_KD = CTL_KD | 0x80;
      static constexpr Byte GET_CTL_ANTI_WIND_UP = CTL_ANTI_WIND_UP | 0x80;
      static constexpr Byte GET_CTL_INPUT_FILTER = CTL_INPUT_FILTER | 0x80;

      static constexpr Byte RESPONSE_MOTOR_RPM = 0xa0;
      static constexpr Byte RESPONSE_MOTOR_POS = 0xa1;
      static constexpr Byte RESPONSE_MOTOR_PARAMETER = 0xa2;
    };
  };
  struct POWER_MANAGEMENT {
    struct MEASUREMENT {
      static constexpr Byte CURRENT = 0x01;
      static constexpr Byte VOLTAGE = 0x02;
    };
  };
  struct LIGHTING {
    struct COMMAND {
      static constexpr Byte BEAT        = 1; // Heartbeat of IOTShield for synchronizing attached devices
      static constexpr Byte LIGHTS_OFF  = 2; // Lights off
      static constexpr Byte DIM_LIGHT   = 3; // Dimmed headlight
      static constexpr Byte HIGH_BEAM   = 4; // high beam headlight
      static constexpr Byte FLASH_ALL   = 5; // Flash lights
      static constexpr Byte FLASH_LEFT  = 6; // Flash lights to the left
      static constexpr Byte FLASH_RIGHT = 7; // Flash lights to the right
      static constexpr Byte PULSATION   = 8; // Pulsation
      static constexpr Byte ROTATION    = 9; // Rotating light
      static constexpr Byte RUNNING     = 10;// Running light
      static constexpr Byte DISTANCE    = 11;// Indicate distance by color
    };
  };
};

} // end namespace message
} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
