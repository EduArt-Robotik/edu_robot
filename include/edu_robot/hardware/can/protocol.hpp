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
      static constexpr Byte TICKS_PER_REVISION = 0x32;
      static constexpr Byte SET_RPM_MAX = 0x04;
    };
  };
  struct POWER_MANAGEMENT {
    struct MEASUREMENT {
      static constexpr Byte CURRENT = 0x01;
      static constexpr Byte VOLTAGE = 0x02;
    };
  };
};

} // end namespace message
} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
