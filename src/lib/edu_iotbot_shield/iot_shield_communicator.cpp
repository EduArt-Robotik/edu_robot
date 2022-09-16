#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

#include <iostream>

namespace eduart {
namespace robot {
namespace iotbot {

IotShieldCommunicator::IotShieldCommunicator(char const* const device_name)
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
  std::function<void(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>&)> callback)
{
  _process_received_bytes = callback;
}

void IotShieldCommunicator::registerDevice(std::shared_ptr<IotShieldDevice> device)
{

}

void IotShieldCommunicator::sendBytes(const std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& bytes)
{
#if _WITH_MRAA
  _uart->write((char*)bytes.data(), bytes.size());
#endif

  receiveBytes();
}

void IotShieldCommunicator::receiveBytes()
{
  std::array<std::uint8_t, UART::BUFFER::RX_SIZE> bytes;

#if _WITH_MRAA
   _uart->read((char*)bytes.data(), bytes.size());
#endif
  _process_received_bytes(bytes);
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
