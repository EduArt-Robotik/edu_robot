/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator.hpp"

#include <cassert>
#include <memory>
#include <mutex>
#include <utility>

namespace eduart {
namespace robot {
namespace hardware {

class CommunicatorNode
{
public:
  CommunicatorNode(std::shared_ptr<Communicator> communicator)
    : _communicator(communicator)
  {
    assert(_communicator == communicator);
  }
  virtual ~CommunicatorNode() {
    // Deactivate all data endpoints.
    for (auto & endpoint : _data_endpoint) {
      endpoint->deactivate();
    }
  }

  inline std::mutex& rxDataMutex() { return _data_mutex; }
  
  template <class EndPointType, class Message, typename ...Args>
  std::shared_ptr<RxDataEndPoint> createRxDataEndPoint(Args&& ...args)
  {
    auto end_point = EndPointType::template make_data_endpoint<Message>(this, std::forward<Args>(args)...);
    _data_endpoint.push_back(end_point);
    _communicator->registerRxDataEndpoint(end_point);

    return end_point;
  }

  template <class Request>
  auto sendRequest(Request&& request) {
    
  }

private:
  std::shared_ptr<Communicator> _communicator;
  std::mutex _data_mutex;
  std::vector<std::shared_ptr<RxDataEndPoint>> _data_endpoint;
  std::chrono::time_point<std::chrono::system_clock> _stamp_sent;
};

} // end namespace hardware
} // end namespace eduart
} // end namespace robot
