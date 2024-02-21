/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"

#include <memory>
#include <mutex>

namespace eduart {
namespace robot {
namespace hardware {

class Communicator;

class CommunicatorTxDevice
{
public:
  CommunicatorTxDevice(std::shared_ptr<Communicator> communicator)
    : _communicator(communicator)
  { }
  virtual ~CommunicatorTxDevice() = default;

protected:
  std::shared_ptr<Communicator> _communicator;
  message::TxMessageDataBuffer _tx_buffer;
};

class CommunicatorRxDevice
{
public:
  CommunicatorRxDevice()
  { }
  virtual ~CommunicatorRxDevice() = default;

  virtual void processRxData(const message::RxMessageDataBuffer& data) = 0;
  inline std::mutex& rxDataMutex() { return _data_mutex; }

private:
  std::mutex _data_mutex;
};

class CommunicatorTxRxDevice : public CommunicatorTxDevice
                             , public CommunicatorRxDevice
{
public:
  CommunicatorTxRxDevice(std::shared_ptr<Communicator> communicator)
    : CommunicatorTxDevice(communicator)
    , CommunicatorRxDevice()
  { }
  ~CommunicatorTxRxDevice() override = default;
};

} // end namespace hardware
} // end namespace eduart
} // end namespace robot
