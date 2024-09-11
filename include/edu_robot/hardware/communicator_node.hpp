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
#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

namespace eduart {
namespace robot {
namespace hardware {

/**
 * \brief Does the communication with the physical hardware. Used by all hardware components at HAL.
 */
class CommunicatorNode
{
public:
  CommunicatorNode(std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
    : _executer(executer)
    , _communicator(communicator)
  {
    assert(_communicator == communicator);
  }
  virtual ~CommunicatorNode() {
    // Stop executer: actually it should be unregistered and not stopped. \todo fix me!
    _executer->stop();

    // Deactivate all data endpoints.
    for (auto & endpoint : _data_endpoint) {
      // \todo it crashes here! fix me!
      endpoint->deactivate();
    }
  }

  inline std::mutex& rxDataMutex() { return _data_mutex; }
  
  template <class EndPointType, class Message, typename ...Args>
  std::shared_ptr<RxDataEndPoint> createRxDataEndPoint(Args&& ...args)
  {
    auto end_point = EndPointType::template make_data_endpoint<Message>(_executer, std::forward<Args>(args)...);
    _data_endpoint.push_back(end_point);
    _communicator->registerRxDataEndpoint(end_point);

    return end_point;
  }

  Request sendRequest(Request&& request, const std::chrono::milliseconds timeout) {
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, timeout);
    return future_response.get();    
  }

  void addSendingJob(std::function<void()> sending_function, const std::chrono::milliseconds time_interval) {
    _executer->addJob(sending_function, time_interval);
  }

private:
  std::shared_ptr<Executer> _executer;
  std::shared_ptr<Communicator> _communicator;
  std::mutex _data_mutex;
  std::vector<std::shared_ptr<RxDataEndPoint>> _data_endpoint;

  std::chrono::time_point<std::chrono::system_clock> _stamp_sent;
};

} // end namespace hardware
} // end namespace eduart
} // end namespace robot
