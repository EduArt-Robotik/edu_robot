/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/ethernet_gateway/udp/message.hpp"

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <edu_robot/hardware/communicator.hpp>

#include <cstdint>
#include <algorithm>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

class EthernetRequest : public hardware::Request
{
public:
  template <class Message, class... Arguments>
  inline static EthernetRequest make_request(Arguments&&... args) {
    return EthernetRequest(Message{}, std::forward<Arguments>(args)...);
  }
  EthernetRequest(EthernetRequest&&) = default;

private:
  template <class CommandByte, class... Elements>
  EthernetRequest(const udp::message::MessageFrame<CommandByte, Elements...>, const typename Elements::type&... element_value) {
    _request_message = udp::message::MessageFrame<CommandByte, Elements...>::serialize(++_sequence_number, element_value...);
    const auto search_pattern = udp::message::MessageFrame<CommandByte, Elements...>::makeSearchPattern(_sequence_number);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }

  static std::uint8_t _sequence_number;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
