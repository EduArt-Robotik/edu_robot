/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator.hpp"

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
  { }
  virtual ~CommunicatorNode() = default;

protected:
  std::shared_ptr<Communicator> _communicator;  
};

class CommunicatorTxNode : virtual public CommunicatorNode
{
public:
  CommunicatorTxNode(std::shared_ptr<Communicator> communicator)
    : CommunicatorNode(communicator)
  { }
  virtual ~CommunicatorTxNode() = default;

protected:
  virtual void doCommunication() = 0;

  // message::TxMessageDataBuffer _tx_buffer;
  std::chrono::time_point<std::chrono::system_clock> _stamp_sent;
};

class CommunicatorRxNode : virtual public CommunicatorNode
{
public:
  CommunicatorRxNode(std::shared_ptr<Communicator> communicator)
    : CommunicatorNode(communicator)
  { }
  virtual ~CommunicatorRxNode() {
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

private:
  std::mutex _data_mutex;
  std::vector<std::shared_ptr<RxDataEndPoint>> _data_endpoint;
};

class CommunicatorTxRxNode : public CommunicatorTxNode
                           , public CommunicatorRxNode
{
public:
  CommunicatorTxRxNode(std::shared_ptr<Communicator> communicator)
    : CommunicatorNode(communicator)    
    , CommunicatorTxNode(communicator)
    , CommunicatorRxNode(communicator)
  { }
  ~CommunicatorTxRxNode() override = default;

protected:
  using CommunicatorTxNode::_communicator;
};

} // end namespace hardware
} // end namespace eduart
} // end namespace robot
