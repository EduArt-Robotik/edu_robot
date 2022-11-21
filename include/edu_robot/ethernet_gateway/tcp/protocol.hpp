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
    };
    struct SET {
      static constexpr Byte MOTOR_CONTROLLER_PARAMETER = 0x11;
      static constexpr Byte ENCODER_PARAMETER = 0x12;
      static constexpr Byte PID_CONTROLLER_PARAMETER = 0x13;
      static constexpr Byte MOTOR_RPM = 0x14;
      static constexpr Byte MOTOR_ENABLE = 0x15;
      static constexpr Byte MOTOR_DISABLE = 0x16;
    };
  };
};

} // end namespace message
} // end namespace tcp
} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
