/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/hardware/iot_shield/uart/message.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldCommunicator;

class IotShieldTxDevice
{
public:
  IotShieldTxDevice(std::shared_ptr<IotShieldCommunicator> communicator)
    : _communicator(communicator)
  { }
  virtual ~IotShieldTxDevice() = default;

protected:
  std::shared_ptr<IotShieldCommunicator> _communicator;
  uart::message::TxMessageDataBuffer _tx_buffer;
};

class IotShieldRxDevice
{
public:
  IotShieldRxDevice() = default;
  virtual ~IotShieldRxDevice() = default;

  virtual void processRxData(const uart::message::RxMessageDataBuffer& data) = 0;
};

class IotShieldTxRxDevice : public IotShieldTxDevice, public IotShieldRxDevice
{
public:
  IotShieldTxRxDevice(std::shared_ptr<IotShieldCommunicator> communicator)
    : IotShieldTxDevice(communicator)
    , IotShieldRxDevice()
  { }
  ~IotShieldTxRxDevice() override = default;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
