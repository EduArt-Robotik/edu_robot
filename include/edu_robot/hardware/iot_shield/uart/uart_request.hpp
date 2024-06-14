/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <edu_robot/hardware/communicator.hpp>

#include <algorithm>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {
namespace uart {

class Request : public hardware::Request
{
public:
  template <class Message, class... Arguments>
  inline static Request make_request(Arguments&&... args) {
    auto request_message = Message::serialize(std::forward<Arguments>(args)...);
    const auto response_message = Message::makeSearchPattern();
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

} // end namespace uart
} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
