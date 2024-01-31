/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can/message.hpp"

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <edu_robot/hardware/communicator.hpp>

#include <cstdint>
#include <algorithm>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

class Request : public hardware::Request
{
public:
  template <class Message, class... Arguments>
  inline static Request make_request(const std::uint16_t can_address, Arguments&&... args) {
    return Request(Message{}, can_address, std::forward<Arguments>(args)...);
  }
  Request(Request&&) = default;

private:
  template <can::message::Byte CommandByte, class... Elements>
  Request(const can::message::MessageFrame<CommandByte, Elements...>, const can::message::Byte can_address, const typename Elements::type&... element_value) {
    _request_message = can::message::MessageFrame<CommandByte, Elements...>::serialize(can_address, element_value...);
    const auto search_pattern = can::message::MessageFrame<CommandByte, Elements...>::makeSearchPattern(can_address);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }
};

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
