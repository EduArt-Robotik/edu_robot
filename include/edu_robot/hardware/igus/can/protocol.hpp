/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/can/message.hpp>

namespace eduart {
namespace robot {
namespace igus {
namespace can {
namespace message {

using hardware::can::message::Byte;

struct PROTOCOL {
  struct BUFFER {

  };
  struct COMMAND {
    static constexpr Byte ENABLE_MOTOR = 0x09;
    static constexpr Byte DISABLE_MOTOR = 0x0a;
    static constexpr Byte RESET = 0x06;

    struct GET {
      static constexpr Byte PARAMETER = 0x03;
    };
    struct SET {
      static constexpr Byte VELOCITY = 0x15;
      static constexpr Byte PARAMETER = 0x02;
      static constexpr Byte COMMAND = 0x01;
    };
  };
  struct MEASUREMENT {
    static constexpr Byte MOTOR_CONTROLLER_RPM = 0x31;
    static constexpr Byte IMU = 0x32;
    static constexpr Byte DISTANCE = 0x33;
  };
  struct ERROR {
    static constexpr Byte BROWN_OUT_OR_WATCH_DOG_BIT = 0;
    static constexpr Byte VELOCITY_LAG_BIT = 1;
    static constexpr Byte MOTOR_NOT_ENABLED = 2;
    static constexpr Byte COMM_WATCH_DOG = 3;
    static constexpr Byte POSITION_LAG = 4;
    static constexpr Byte ENCODER_ERROR = 5;
    static constexpr Byte OVER_CURRENT = 6;
    static constexpr Byte CAN_ERROR = 7;
  };
};

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace eduart
} // end namespace robot
