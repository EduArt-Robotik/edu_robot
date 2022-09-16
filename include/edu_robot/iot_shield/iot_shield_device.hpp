/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart_message_conversion.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldCommunicator;

class IotShieldDevice
{
public:
  IotShieldDevice(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator)
    : _name(name)
    , _id(id)
    , _communicator(communicator)
  { }
  virtual ~IotShieldDevice() = default;

  const std::string& name() const { return _name; }
  std::uint8_t id() const { return _id; }

private:
  std::string _name;
  std::uint8_t _id;

protected:
  std::shared_ptr<IotShieldCommunicator> _communicator;
  std::array<std::uint8_t, UART::BUFFER::TX_SIZE> _tx_buffer;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
