/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/message.hpp"

#include <edu_robot/hardware/communicator.hpp>

#include <cstdint>
#include <algorithm>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {

using hardware::igus::can::message::MessageFrame;
using hardware::igus::can::message::Byte;
using hardware::can::message::element::Command;

class CanRequest : public hardware::Request
{
public:
  template <class Message, class... Arguments>
  inline static CanRequest make_request(const std::uint32_t can_address, Arguments&&... args) {
    return CanRequest(Message{}, can_address, std::forward<Arguments>(args)...);
  }
  CanRequest(CanRequest&&) = default;

private:
  template <Byte CommandByte, class... Elements>
  CanRequest(MessageFrame<CommandByte, Elements...>, const std::uint32_t can_address, const typename Elements::type&... element_value) {
    _request_message = MessageFrame<CommandByte, Elements...>::serialize(can_address, element_value...);
    const auto search_pattern = MessageFrame<CommandByte, Elements...>::makeSearchPattern(can_address);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }
  // special case for two commands
  template <Byte FirstCommand, Byte SecondCommand, class... Elements>
  CanRequest(MessageFrame<FirstCommand, Command<SecondCommand>, Elements...>, const std::uint32_t can_address, const typename Elements::type&... element_value) {
    _request_message = MessageFrame<FirstCommand, Command<SecondCommand>, Elements...>::serialize(can_address, 0, element_value...);
    const auto search_pattern = MessageFrame<FirstCommand, Command<SecondCommand>, Elements...>::makeSearchPattern(can_address);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }
};

} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
