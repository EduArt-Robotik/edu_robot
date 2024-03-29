/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/protocol.hpp"
#include "edu_robot/hardware/igus/can/message.hpp"

#include <edu_robot/rpm.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

using hardware::can::message::RxMessageDataBuffer;
using hardware::can::message::element::Uint8;
using hardware::can::message::element::Command;
using hardware::igus::can::message::element::VelocityCanAddress;
using hardware::igus::can::message::element::CommandCanAddress;

using SetEnableMotor = MessageFrame<CommandCanAddress,
                                    PROTOCOL::COMMAND::SET::COMMAND,
                                    Command<PROTOCOL::COMMAND::ENABLE_MOTOR>>;
using SetDisableMotor = MessageFrame<CommandCanAddress,
                                     PROTOCOL::COMMAND::SET::COMMAND,
                                     Command<PROTOCOL::COMMAND::DISABLE_MOTOR>>;
using SetReset = MessageFrame<CommandCanAddress,
                              PROTOCOL::COMMAND::SET::COMMAND,
                              Command<PROTOCOL::COMMAND::RESET>>;
using SetVelocity = MessageFrame<VelocityCanAddress,
                                 PROTOCOL::COMMAND::SET::VELOCITY,
                                 element::Velocity,  // velocity in ?
                                 Uint8>;             // timestamp in ?

struct AcknowledgedVelocity : public MessageFrame<VelocityCanAddress,
                                                  PROTOCOL::COMMAND::SET::VELOCITY,
                                                  Uint8,    // error code
                                                  element::Velocity, // measured velocity ?
                                                  Uint8,    // timestamp in ?
                                                  Uint8,    // shunt in ?
                                                  Uint8>    // digital input
{
  inline static constexpr std::uint8_t canId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
  inline static constexpr std::uint8_t errorCode(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr Rpm rpm(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline static constexpr std::uint8_t timestamp(const RxMessageDataBuffer &rx_buffer) {
    return deserialize<3>(rx_buffer);
  }
  inline static constexpr std::uint8_t shunt(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<4>(rx_buffer);
  }
  inline static constexpr std::uint8_t digitalInput(const RxMessageDataBuffer &rx_buffer) {
    return deserialize<5>(rx_buffer);
  }
  inline static constexpr bool enabled(const RxMessageDataBuffer& rx_buffer) {
    return (errorCode(rx_buffer) & PROTOCOL::ERROR::MOTOR_NOT_ENABLED) == false;
  }
};

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
