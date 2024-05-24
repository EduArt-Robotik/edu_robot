/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can_gateway/can/message.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"
#include "edu_robot/hardware/igus/can/message.hpp"

#include <cstdint>
#include <edu_robot/rpm.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

using hardware::can_gateway::can::message::RxMessageDataBuffer;
using hardware::can_gateway::can::message::element::Uint8;
using hardware::can_gateway::can::message::element::Uint16;
using hardware::can_gateway::can::message::element::Int32;
using hardware::can_gateway::can::message::element::Command;
using hardware::igus::can::message::element::VelocityCanAddress;
using hardware::igus::can::message::element::CommandCanAddress;

template <typename Type, Type Data>
using ConstDataField = hardware::can_gateway::can::message::element::impl::ConstDataField<Type, Data>;

// Parameters
template <Byte Parameter>
using SetParameterUint8 = MessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>,
                                       Parameter,
                                       Uint8,
                                       ConstDataField<Byte, 0x00>>;

template <Byte Parameter>
using SetParameterUint16 = MessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>,
                                        Parameter,
                                        Uint16>;

using SetMaxMissCommunication = SetParameterUint16<PROTOCOL::PARAMETER::MAX_MISSED_COMMUNICATION>;
using SetMaxLag = SetParameterUint16<PROTOCOL::PARAMETER::MAX_LAG>;

// Operations
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

struct AcknowledgedVelocity : public Message<VelocityCanAddress,
                                             Uint8, // error code
                                             Int32, // measured velocity ?
                                             Uint8, // timestamp in ?
                                             Uint8, // shunt in ?
                                             Uint8> // digital input
{
  inline static constexpr std::uint32_t canId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
  inline static constexpr std::uint8_t errorCode(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr std::int32_t position(const RxMessageDataBuffer& rx_buffer) {
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
