/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/ethernet_gateway/udp/message.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {

class Communicator;

namespace ethernet {

class EthernetGatewayTxDevice
{
public:
  EthernetGatewayTxDevice(std::shared_ptr<Communicator> communicator)
    : _communicator(communicator)
  { }
  virtual ~EthernetGatewayTxDevice() = default;

protected:
  std::shared_ptr<Communicator> _communicator;
  message::TxMessageDataBuffer _tx_buffer;
};

class EthernetGatewayRxDevice
{
public:
  EthernetGatewayRxDevice()
  { }
  virtual ~EthernetGatewayRxDevice() = default;

  virtual void processRxData(const message::RxMessageDataBuffer& data) = 0;
};

class EthernetGatewayTxRxDevice : public EthernetGatewayTxDevice, public EthernetGatewayRxDevice
{
public:
  EthernetGatewayTxRxDevice(std::shared_ptr<Communicator> communicator)
    : EthernetGatewayTxDevice(communicator)
    , EthernetGatewayRxDevice()
  { }
  ~EthernetGatewayTxRxDevice() override = default;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
