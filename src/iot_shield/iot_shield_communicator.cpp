#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

namespace eduart {
namespace robot {
namespace iot_bot {

IotShieldCommunicator::IotShieldCommunicator()
{

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

}

void IotShieldCommunicator::receiveBytes(std::vector<std::uint8_t>& bytes)
{

}

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
