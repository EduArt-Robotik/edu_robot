/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/tcp/message.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class EthernetGatewayTxDevice
{
public:
  EthernetGatewayTxDevice(std::shared_ptr<EthernetCommunicator> communicator)
    : _communicator(communicator)
  { }
  virtual ~EthernetGatewayTxDevice() = default;

protected:
  std::shared_ptr<EthernetCommunicator> _communicator;
  tcp::message::TxMessageDataBuffer _tx_buffer;
};

class EthernetGatewayRxDevice
{
public:
  EthernetGatewayRxDevice()
  { }
  virtual ~EthernetGatewayRxDevice() = default;

  virtual void processRxData(const tcp::message::RxMessageDataBuffer& data) = 0;
};

class EthernetGatewayTxRxDevice : public EthernetGatewayTxDevice, public EthernetGatewayRxDevice
{
public:
  EthernetGatewayTxRxDevice(std::shared_ptr<EthernetCommunicator> communicator)
    : EthernetGatewayTxDevice(communicator)
    , EthernetGatewayRxDevice()
  { }
  ~EthernetGatewayTxRxDevice() override = default;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
