/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <edu_robot/hardware/can/message.hpp>
#include <edu_robot/rpm.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

namespace element {

// Specialize 
using hardware::can::message::element::impl::DataField;
using hardware::can::message::element::Command;

struct Velocity : public DataField<std::uint8_t> {
  inline static constexpr std::array<hardware::can::message::Byte, size()> serialize(const Rpm value) {
    return hardware::can::message::element::impl::DataField<std::uint8_t>::serialize(
      static_cast<std::uint8_t>(value + 127.0)
    ); // 127 is zero
  }
  inline static constexpr Rpm deserialize(const hardware::can::message::Byte data[size()]) {
    return Rpm(hardware::can::message::element::impl::DataField<std::uint8_t>::deserialize(data) - 127); // 127 is zero
  }
};

struct CanAddress : public hardware::can::message::element::CanAddress {// DataField<std::uint32_t> {
  inline static constexpr std::array<Byte, CanAddress::size()> makeSearchPattern(const std::uint32_t can_address) {
    return DataField<std::uint32_t>::serialize(can_address | 0x01);
  }
};

} // end namespace element

using hardware::can::message::Message;
using hardware::message::TxMessageDataBuffer;
using hardware::message::RxMessageDataBuffer;
using hardware::can::message::element::impl::make_message_search_pattern;

template <Byte CommandByte, class ...Elements>
struct MessageFrame : public Message<element::CanAddress, element::Command<CommandByte>, Elements...>
{
private:
  using MessageType = Message<element::CanAddress, element::Command<CommandByte>, Elements...>;

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
};

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
