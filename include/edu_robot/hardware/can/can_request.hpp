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
  inline static Request make_request(const std::uint32_t can_address, Arguments&&... args) {
    return Request(Message{}, can_address, std::forward<Arguments>(args)...);
  }
  Request(Request&&) = default;

private:
  // template <class... Elements>
  // Request(const can::message::MessageFrame<Elements...>, const can::message::Byte can_address, const typename Elements::type&... element_value) {
  //   _request_message = can::message::MessageFrame<Elements...>::serialize(can_address, element_value...);
  //   const auto search_pattern = can::message::MessageFrame<Elements...>::makeSearchPattern(can_address);

  //   _response_search_pattern.resize(search_pattern.size());
  //   std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  // }
  template <template <class...> class Message, class... Elements>
  Request(const Message<Elements...>, const std::uint32_t can_address, const typename Elements::type&... element_value) {
    _request_message = Message<Elements...>::serialize(can_address, element_value...);
    const auto search_pattern = Message<Elements...>::makeSearchPattern(can_address);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }  
};

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
