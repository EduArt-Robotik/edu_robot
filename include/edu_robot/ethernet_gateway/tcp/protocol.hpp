/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstdint>

namespace eduart {
namespace robot {
namespace ethernet {
namespace tcp {
namespace message {

using Byte = std::uint8_t;

struct PROTOCOL {
  struct BUFFER {
    static constexpr Byte START_BYTE = 0xff;
    static constexpr Byte END_BYTE   = 0xee;
  };
  struct COMMAND {
    struct GET {
      static constexpr Byte FIRMWARE_VERSION = 0x01;
      static constexpr Byte IMU_MEASUREMENT = 0x02;
      static constexpr Byte STATUS = 0x03;
      static constexpr Byte DISTANCE_MEASUREMENT = 0x04;
    };
    struct SET {
      static constexpr Byte MOTOR_CONTROLLER_PARAMETER = 0x11;
      static constexpr Byte ENCODER_PARAMETER = 0x12;
      static constexpr Byte PID_CONTROLLER_PARAMETER = 0x13;
      static constexpr Byte MOTOR_RPM = 0x14;
      static constexpr Byte MOTOR_ENABLE = 0x15;
      static constexpr Byte MOTOR_DISABLE = 0x16;
      static constexpr Byte MOTOR_MEASUREMENT = 0x17;
      static constexpr Byte IMU_MEASUREMENT = 0x18;
      static constexpr Byte DISTANCE_SENSOR_MEASUREMENT = 0x19;
      static constexpr Byte DISABLE_ALL_MEASUREMENTS = 0x1a;
      static constexpr Byte LIGHTING_COLOR_AND_MODE = 0x1b;
      static constexpr Byte IMU_PARAMETER = 0x1c;
    };
  };
  struct MEASUREMENT {
    static constexpr Byte MOTOR_CONTROLLER_RPM = 0x31;
    static constexpr Byte IMU = 0x32;
    static constexpr Byte DISTANCE = 0x33;
  };
  struct LIGHTING {
    struct MODE {
      static constexpr Byte LIGHTS_OFF = 2;   // Lights off
      static constexpr Byte DIM_LIGHT  = 3;   // Dimmed headlight
      static constexpr Byte HIGH_BEAM  = 4;   // high beam headlight
      struct FLASH {
        static constexpr Byte ALL   = 5;   // Flash lights
        static constexpr Byte LEFT  = 6;   // Flash lights to the left
        static constexpr Byte RIGHT = 7;   // Flash lights to the right
      };
      static constexpr Byte PULSATION = 8;   // Pulsation
      static constexpr Byte ROTATION  = 9;   // Rotating light
      static constexpr Byte RUNNING   = 10;  // Running light
    };
  };
};

} // end namespace message
} // end namespace tcp
} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
