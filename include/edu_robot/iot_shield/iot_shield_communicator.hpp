/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart_message_conversion.hpp"

#if _WITH_MRAA
#include <mraa/common.hpp>
#include <mraa/uart.hpp>
#endif

#include <functional>
#include <memory>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldDevice;



class IotShieldCommunicator
{
public:
  IotShieldCommunicator(char const* const device_name);

  void registerDevice(std::shared_ptr<IotShieldDevice> device);
  void registerProcessReceivedBytes(std::function<void(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>&)> callback);
  void sendBytes(const std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& bytes);

private:
  void receiveBytes();

  std::function<void(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>&)> _process_received_bytes;

#if _WITH_MRAA
  std::unique_ptr<mraa::Uart> _uart;
#endif
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
