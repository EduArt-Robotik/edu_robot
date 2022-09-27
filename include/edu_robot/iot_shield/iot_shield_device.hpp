/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/uart/uart_message_conversion.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldCommunicator;

class IotShieldDevice
{
protected:
  IotShieldDevice(const std::string& name, const std::uint8_t id)
    : _name(name)
    , _id(id)
  { }

public:
  virtual ~IotShieldDevice() = default;

  const std::string& name() const { return _name; }
  std::uint8_t id() const { return _id; }

private:
  std::string _name;
  std::uint8_t _id; // \todo id has actually no meaning, maybe remove it
};

class IotShieldTxDevice : public virtual IotShieldDevice
{
public:
  IotShieldTxDevice(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator)
    : IotShieldDevice(name, id)
    , _communicator(communicator)
  { }
  ~IotShieldTxDevice() override = default;

protected:
  std::shared_ptr<IotShieldCommunicator> _communicator;
  uart::message::TxMessageDataBuffer _tx_buffer;
};

class IotShieldRxDevice : public virtual IotShieldDevice
{
public:
  IotShieldRxDevice(const std::string& name, const std::uint8_t id)
    : IotShieldDevice(name, id)
  { }
  ~IotShieldRxDevice() override = default;

  virtual void processRxData(const uart::message::RxMessageDataBuffer& data) = 0;
};

class IotShieldTxRxDevice : public IotShieldTxDevice, public IotShieldRxDevice
{
public:
  IotShieldTxRxDevice(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator)
    : IotShieldTxDevice(name, id, communicator)
    , IotShieldRxDevice(name, id)
  { }
  ~IotShieldTxRxDevice() override = default;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
