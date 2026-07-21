/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <edu_robot/hardware/communicator.hpp>

#include <cstdint>
#include <algorithm>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
namespace can {

class Request : public hardware::Request
{
public:
  template <class Message, class... Arguments>
  inline static Request make_request(const std::uint32_t can_address, Arguments&&... args) {
    auto request_message = Message::serialize(can_address, std::forward<Arguments>(args)...);
    const auto response_message = Message::makeSearchPattern(can_address);
    return Request(request_message, response_message);
  }
  template <class Message, class... Arguments>
  inline static Request make_request_with_response(
    const std::uint32_t can_address_device, const std::uint32_t can_address_host, Arguments&&... args)
 {
    auto request_message = Message::serialize(can_address_device, std::forward<Arguments>(args)...);
    const auto response_message = Message::makeSearchPattern(can_address_host);
    return Request(request_message, response_message);
  }  
  Request(Request&&) = default;

private:
  template<class SearchPattern>
  Request(message::TxMessageDataBuffer& request_message, const SearchPattern& search_pattern)
  {
    _request_message = std::move(request_message);
    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }
};

} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
