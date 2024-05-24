/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <edu_robot/hardware/can_gateway/can/message.hpp>
#include <edu_robot/rpm.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

namespace element {

// Specialize 

using hardware::can_gateway::can::message::element::impl::DataField;
using hardware::can_gateway::can::message::element::Command;
using hardware::can_gateway::can::message::Byte;
using hardware::can_gateway::can::message::element::Uint8;
using hardware::can_gateway::can::message::element::Int32;

struct Velocity : public Uint8 {
  // inline static constexpr std::array<Byte, size()> serialize(const Rpm value) {
  //   return Uint8::serialize(
  //     static_cast<std::uint8_t>(value + 127.0f)
  //   ); // 127 is zero
  // }
  // inline static constexpr Rpm deserialize(const Byte data[size()]) {
  //   return Rpm(Uint8::deserialize(data) - 127); // 127 is zero
  // }
};

struct Position : public Int32 {

};

struct VelocityCanAddress : public hardware::can_gateway::can::message::element::CanAddress {
  inline static constexpr std::array<Byte, CanAddress::size()> makeSearchPattern(const std::uint32_t can_address) {
    return DataField<std::uint32_t, false>::serialize(can_address | 0x01);
  }
};

struct CommandCanAddress : public hardware::can_gateway::can::message::element::CanAddress {
  inline static constexpr std::array<Byte, CanAddress::size()> makeSearchPattern(const std::uint32_t can_address) {
    return DataField<std::uint32_t, false>::serialize(can_address | 0x02);
  }
};

} // end namespace element

using hardware::can_gateway::can::message::Message;
using hardware::message::TxMessageDataBuffer;
using hardware::message::RxMessageDataBuffer;
using hardware::can_gateway::can::message::element::impl::make_message_search_pattern;

template <class CanAddress, Byte CommandByte, class ...Elements>
struct MessageFrame : public Message<CanAddress, element::Command<CommandByte>, Elements...>
{
private:
  using MessageType = Message<CanAddress, element::Command<CommandByte>, Elements...>;

public:
  using MessageType::size;

  inline static TxMessageDataBuffer serialize(
    const std::uint32_t can_address, const typename Elements::type&... element_value)
  {
    return MessageType::serialize(can_address, 0, element_value...);
  }
  inline constexpr static auto makeSearchPattern(const Byte can_address) {
    return make_message_search_pattern<0>(can_address, MessageType{});
  }
  template <std::size_t Index>
  inline constexpr static auto deserialize(const RxMessageDataBuffer& rx_buffer) {
    return MessageType::template deserialize<Index>(rx_buffer);
  }
  inline static constexpr std::uint8_t canId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }  
};

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
