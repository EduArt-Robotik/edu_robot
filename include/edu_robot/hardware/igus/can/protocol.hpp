/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"
#include <edu_robot/hardware/can_gateway/can/message.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

using hardware::can_gateway::can::message::Byte;

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
  struct PARAMETER {
    // constants for setting parameter
    static constexpr Byte MAX_MISSED_COMMUNICATION = 0x30;
    static constexpr Byte MAX_LAG = 0x31;
    static constexpr Byte MAX_CURRENT = 0x32;
    static constexpr Byte ROLLOVER_FLAG = 0x66;
    static constexpr Byte TIC_SCALE_FACTOR = 0x69;

    // constants for getting parameter
    static constexpr Byte GENERAL = 0x50;
    static constexpr Byte POSITION_CONTROL_LOOP = 0x51;
    static constexpr Byte VELOCITY_CONTROL_LOOP = 0x52;
    static constexpr Byte WORKING_HOURS = 0x54;
    static constexpr Byte SUPPLY_VOLTAGE = 0x55;
    static constexpr Byte FLAGS = 0x59;

    struct PID {
      struct POSITION {
        static constexpr Byte KP = 0x40;
        static constexpr Byte KI = 0x41;
        static constexpr Byte KD = 0x42;
        static constexpr Byte ANTI_WIND_UP = 0x43;
      };
      struct VELOCITY {
        static constexpr Byte KP = 0x44;
        static constexpr Byte KI = 0x45;
        static constexpr Byte KD = 0x46;
        static constexpr Byte ANTI_WIND_UP = 0x47;
      };
    };
  };
};

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
