#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

IotShieldCommunicator::IotShieldCommunicator(char const* const device_name)
  : _wait_time_after_sending(8ms)
  , _last_sending(std::chrono::system_clock::now())
{
  // \todo check for better logging instance.
#if _WITH_MRAA
  std::cout << "open uart device \"" << device_name << "\"" << std::endl;
   _uart = std::make_unique<mraa::Uart>(device_name);

   if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
      std::cerr << "Error setting parity on UART" << std::endl;
   }

   if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
      std::cerr << "Error setting flow control UART" << std::endl;
   }
   
   _uart->flush();
#else
   std::cerr << "UART interface not available. MRAA is missing!" << std::endl;
#endif
}

void IotShieldCommunicator::registerProcessReceivedBytes(
  std::function<void(const std::array<std::uint8_t, uart::message::UART::BUFFER::RX_SIZE>&)> callback)
{
  _process_received_bytes = callback;
}

void IotShieldCommunicator::registerDevice(std::shared_ptr<IotShieldDevice> device)
{

}

void IotShieldCommunicator::sendBytes(const std::array<std::uint8_t, uart::message::UART::BUFFER::TX_SIZE>& bytes)
{
  // Wait a minimum time until next message can be sent.
  // \todo Try to find a non blocking/sleeping solution.
  auto now = std::chrono::system_clock::now();

  while (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_sending) < _wait_time_after_sending) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1ms));
    now = std::chrono::system_clock::now();
  }

#if _WITH_MRAA
  _uart->write((char*)bytes.data(), bytes.size());
  std::cout << "send data: ";
  for (const auto& byte : bytes) {
    std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  }
  std::cout << std::endl;
#endif

  receiveBytes();
}

void IotShieldCommunicator::receiveBytes()
{
  std::array<std::uint8_t, uart::message::UART::BUFFER::RX_SIZE> bytes;

#if _WITH_MRAA
   _uart->read((char*)bytes.data(), bytes.size());
#endif
  std::cout << "received data: ";
  for (const auto& byte : bytes) {
    std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  }
  std::cout << std::endl;
  _process_received_bytes(bytes);
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
