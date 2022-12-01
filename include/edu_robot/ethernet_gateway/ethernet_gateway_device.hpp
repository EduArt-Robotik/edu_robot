/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/tcp/message.hpp"

#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class EthernetGatewayDevice
{
protected:
  EthernetGatewayDevice(const std::string& name)
    : _name(name)
  { }

public:
  virtual ~EthernetGatewayDevice() = default;

  const std::string& name() const { return _name; }

private:
  std::string _name;
};

class EthernetGatewayTxDevice : public virtual EthernetGatewayDevice
{
public:
  EthernetGatewayTxDevice(const std::string& name, std::shared_ptr<EthernetCommunicator> communicator)
    : EthernetGatewayDevice(name)
    , _communicator(communicator)
  { }
  ~EthernetGatewayTxDevice() override = default;

protected:
  std::shared_ptr<EthernetCommunicator> _communicator;
  tcp::message::TxMessageDataBuffer _tx_buffer;
};

class EthernetGatewayRxDevice : public virtual EthernetGatewayDevice
{
public:
  EthernetGatewayRxDevice(const std::string& name)
    : EthernetGatewayDevice(name)
  { }
  ~EthernetGatewayRxDevice() override = default;

  virtual void processRxData(const tcp::message::RxMessageDataBuffer& data) = 0;
};

class EthernetGatewayTxRxDevice : public EthernetGatewayTxDevice, public EthernetGatewayRxDevice
{
public:
  EthernetGatewayTxRxDevice(const std::string& name, std::shared_ptr<EthernetCommunicator> communicator)
    : EthernetGatewayTxDevice(name, communicator)
    , EthernetGatewayRxDevice(name)
  { }
  ~EthernetGatewayTxRxDevice() override = default;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
