/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/message.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace igus {

class CanCommunicator;

class CanGatewayTxDevice
{
public:
  CanGatewayTxDevice(std::shared_ptr<CanCommunicator> communicator)
    : _communicator(communicator)
  { }
  virtual ~CanGatewayTxDevice() = default;

protected:
  std::shared_ptr<CanCommunicator> _communicator;
  can::message::TxMessageDataBuffer _tx_buffer;
};

class CanGatewayRxDevice
{
public:
  CanGatewayRxDevice()
  { }
  virtual ~CanGatewayRxDevice() = default;

  virtual void processRxData(const can::message::RxMessageDataBuffer& data) = 0;
};

class CanGatewayTxRxDevice : public CanGatewayTxDevice, public CanGatewayRxDevice
{
public:
  CanGatewayTxRxDevice(std::shared_ptr<CanCommunicator> communicator)
    : CanGatewayTxDevice(communicator)
    , CanGatewayRxDevice()
  { }
  ~CanGatewayTxRxDevice() override = default;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
