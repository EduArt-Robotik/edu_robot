#include "edu_robot/hardware/iot_shield/uart_communication_device.hpp"
#include <cstddef>
#include <cstdint>
#include <edu_robot/hardware_error.hpp>

#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using namespace std::chrono_literals;

UartCommunicationDevice::UartCommunicationDevice(char const* const device_name)
{
  // \todo check for better logging instance.
#if _WITH_MRAA
  RCLCPP_INFO(rclcpp::get_logger("UartCommunicationDevice"), "open uart device %s.", device_name);
  _uart = std::make_unique<mraa::Uart>(device_name);

  if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
    throw HardwareError(State::UART_HARDWARE_ERROR, "UartCommunicationDevice: error during setting baudrate to 115200.");
  }

  if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
    throw HardwareError(State::UART_HARDWARE_ERROR, "UartCommunicationDevice: error setting parity.");
  }

  if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
    throw HardwareError(State::UART_HARDWARE_ERROR, "UartCommunicationDevice: error setting flow control.");
  }
   
  _uart->flush();
  while (_uart->dataAvailable(1) == true) { char buffer; _uart->read(&buffer, 1); }
#else
  (void)device_name;
  throw HardwareError(State::UART_HARDWARE_ERROR, "UartCommunicationDevice: UART interface not available. MRAA is missing!");
#endif
}

UartCommunicationDevice::~UartCommunicationDevice()
{
#if _WITH_MRAA
  RCLCPP_INFO(rclcpp::get_logger("UartCommunicationDevice"), "closing uart device");
  _uart->flush();
  _uart->close();
#endif
}

void UartCommunicationDevice::send(message::Byte const *const tx_buffer, const std::size_t length)
{
  std::cout << "sending: " << std::hex;
  for (std::size_t i = 0; i < length; ++i) std::cout << static_cast<int>(tx_buffer[i]) << " ";
  std::cout << std::dec << std::endl;

#if _WITH_MRAA
  const char* data = static_cast<const char*>(static_cast<const void*>(tx_buffer));
  const std::size_t sent_bytes = _uart->write(data, length);
  
  // DEBUG BEGIN    
  // std::cout << "send data: ";
  // for (const auto& byte : tx_buffer) {
  //   std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END

  if (sent_bytes != length) {
    throw HardwareError(State::UART_SENDING_FAILED, "Less byte sent as expected.");
  }

#else
  (void)tx_buffer;
  (void)length;
#endif
}

message::RxMessageDataBuffer UartCommunicationDevice::receive()
{
  constexpr std::size_t rx_message_byte_count = 32;
  message::RxMessageDataBuffer rx_buffer(rx_message_byte_count);
  std::size_t received_bytes = 0;

#if _WITH_MRAA
  while (received_bytes < rx_buffer.size() && _uart->dataAvailable(100)) {
    received_bytes += _uart->read(
      static_cast<char*>(static_cast<void*>(rx_buffer.data())) + received_bytes, 1
    );
  }
#endif

  // DEBUG BEGIN
  // std::cout << "received data: ";
  // for (const auto& byte : rx_buffer) {
  //   std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END

  if (received_bytes == 0) {
    // return rx_buffer because of RVO
    rx_buffer.clear();
    return rx_buffer;
  }
  else if (received_bytes != rx_buffer.size()) {
  std::cout << "received: " << std::hex;
  for (const auto byte : rx_buffer) std::cout << static_cast<int>(byte) << " ";
  std::cout << std::dec << std::endl;
    throw HardwareError(State::UART_RECEIVING_FAILED, "Received bytes do not fit to rx buffer.");
  }
  // else: --> all fine
  std::cout << "received: " << std::hex;
  for (const auto byte : rx_buffer) std::cout << static_cast<int>(byte) << " ";
  std::cout << std::dec << std::endl;

  return rx_buffer;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
