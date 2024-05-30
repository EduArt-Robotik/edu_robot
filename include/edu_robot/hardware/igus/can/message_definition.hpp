/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can_gateway/can/message.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"
#include "edu_robot/hardware/igus/can/message.hpp"
#include "edu_robot/hardware/message_buffer.hpp"

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
using hardware::can_gateway::can::message::element::CanAddress;
using hardware::can_gateway::can::message::NoResponseMessageFrame;
using hardware::igus::can::message::element::VelocityCanAddress;
using hardware::igus::can::message::element::CommandCanAddress;
using hardware::igus::can::message::element::ControllerParameter;

template <typename Type, Type Data>
using ConstDataField = hardware::can_gateway::can::message::element::impl::ConstDataField<Type, Data>;

// Parameters
template <Byte Parameter, Byte FillByte = 0x00>
struct SetParameterUint8 : public NoResponseMessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>,
                                                         ConstDataField<Byte, Parameter>,
                                                         Uint8,
                                                         ConstDataField<Byte, FillByte>>
{
  inline static TxMessageDataBuffer serialize(const std::uint32_t can_address, const Uint8::type value) {
    using MessageType = NoResponseMessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>, ConstDataField<Byte, Parameter>, Uint8, ConstDataField<Byte, FillByte>>;
    return MessageType::serialize(can_address, 0, 0, value, 0);
  }
};

template <Byte Parameter>
struct SetParameterUint16 : public NoResponseMessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>,
                                                          ConstDataField<Byte, Parameter>,
                                                          Uint16>
{
  inline static TxMessageDataBuffer serialize(const std::uint32_t can_address, const Uint16::type value) {
    using MessageType = NoResponseMessageFrame<Command<PROTOCOL::COMMAND::SET::PARAMETER>, ConstDataField<Byte, Parameter>, Uint16>;
    return MessageType::serialize(can_address, 0, 0, value);
  }
};

// Set Parameter
using SetMaxMissCommunication = SetParameterUint16<PROTOCOL::PARAMETER::MAX_MISSED_COMMUNICATION>;
using SetMaxLag = SetParameterUint16<PROTOCOL::PARAMETER::MAX_LAG>;
using SetMaxCurrent = SetParameterUint8<PROTOCOL::PARAMETER::MAX_CURRENT>;
using SetPidVelocityKp = SetParameterUint16<PROTOCOL::PARAMETER::PID::VELOCITY::KP>;
using SetPidVelocityKi = SetParameterUint16<PROTOCOL::PARAMETER::PID::VELOCITY::KI>;
using SetPidVelocityKd = SetParameterUint16<PROTOCOL::PARAMETER::PID::VELOCITY::KD>;
using SetRolloverFlag = SetParameterUint8<PROTOCOL::PARAMETER::ROLLOVER_FLAG, 0x9b>;
using SetTicScaleFactor = SetParameterUint8<PROTOCOL::PARAMETER::TIC_SCALE_FACTOR, 0x9e>;

// Get Parameter
template <Byte Parameter>
struct GetParameter : public MessageFrame<CommandCanAddress,
                                          PROTOCOL::COMMAND::GET::PARAMETER,
                                          ConstDataField<Byte, Parameter>>
{
private:
  using MessageType = MessageFrame<CommandCanAddress, PROTOCOL::COMMAND::GET::PARAMETER, ConstDataField<Byte, Parameter>>;

public:
  inline static TxMessageDataBuffer serialize(const std::uint32_t can_address) {
    return MessageType::serialize(can_address, 0);
  }
  inline constexpr static auto makeSearchPattern(const Byte can_address) {
    return make_message_search_pattern<0, 2>(can_address, MessageType{});
  }
};

using GetGeneralParameter = GetParameter<PROTOCOL::PARAMETER::GENERAL>;
using GetVelocityControlParameter = GetParameter<PROTOCOL::PARAMETER::VELOCITY_CONTROL_LOOP>;
using GetWorkingHours = GetParameter<PROTOCOL::PARAMETER::WORKING_HOURS>;
using GetSupplyVoltage = GetParameter<PROTOCOL::PARAMETER::SUPPLY_VOLTAGE>;
using GetFlags = GetParameter<PROTOCOL::PARAMETER::FLAGS>;

struct GeneralParameter : public Message<CommandCanAddress, // can address
                                         Uint8,             // parameter id
                                         Uint16,            // max missed communication
                                         Uint16,            // max lag
                                         Uint8,             // max current
                                         Uint16>            // max lag velocity
{
  inline static constexpr std::uint8_t parameterId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr std::uint16_t maxMissedCommunication(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline static constexpr std::uint16_t maxLag(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<3>(rx_buffer);
  }
  inline static constexpr std::uint8_t maxCurrent(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<4>(rx_buffer);
  }
  inline static constexpr std::uint16_t maxLagVelocity(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<5>(rx_buffer);
  }
};

struct VelocityControlLoopParameter : public Message<CommandCanAddress,   // can address
                                                     Uint8,               // parameter id
                                                     ControllerParameter, // pid velocity kp
                                                     ControllerParameter, // pid velocity ki
                                                     ControllerParameter, // pid velocity kd
                                                     Uint8>               // anti wind up flag
{
  inline static constexpr std::uint8_t parameterId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  // inline static constexpr float velocityControllerKp
};

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
                                             Int32, // measured position in ?
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
