/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <functional>
#include <memory>
#include <array>

namespace eduart {
namespace robot {
namespace iot_bot {

class IotShieldDevice;

struct UART {
  struct BUFFER {
    static constexpr std::size_t TX_SIZE = 11;
    static constexpr std::size_t RX_SIZE = 32;
  
    static constexpr std::uint8_t START_BYTE = 0xff;
    static constexpr std::uint8_t END_BYTE   = 0xee;
  };
  struct COMMAND {
    static constexpr std::uint8_t ENABLE  = 0x01;
    static constexpr std::uint8_t DISABLE = 0x02;
  };
};

class IotShieldCommunicator
{
public:
  IotShieldCommunicator();

  void registerDevice(std::shared_ptr<IotShieldDevice> device);
  void registerProcessReceivedBytes(std::function<void(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>&)> callback);
  void sendBytes(const std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& bytes);

private:
  void receiveBytes(std::vector<std::uint8_t>& bytes);

  std::array<std::uint8_t, UART::BUFFER::RX_SIZE> _rx_buffer; 
  std::function<void(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>&)> _process_received_bytes;
};

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
