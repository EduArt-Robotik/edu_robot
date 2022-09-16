#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart_message_conversion.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <memory>
#include <functional>
#include <iostream>

namespace eduart {
namespace robot {
namespace iot_bot {

IotShield::IotShield(char const* const device_name)
  : _communicator(std::make_shared<IotShieldCommunicator>())
{
  _communicator->registerProcessReceivedBytes(
    std::bind(&IotShield::processStatusReport, this, std::placeholders::_1)
  );

  // \todo check for better logging instance.
#if _WITH_MRAA
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

IotShield::~IotShield()
{
  // \todo do some clean up on hardware side!
}

void IotShield::enable()
{
  // clear buffer
  _tx_buffer = { 0 };

  // prepare message and send it
  _tx_buffer[0]  = UART::BUFFER::START_BYTE;
  _tx_buffer[1]  = UART::COMMAND::ENABLE;
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer); 
}

void IotShield::disable()
{
  // clear buffer
  _tx_buffer = { 0 };

  // prepare message and send it
  _tx_buffer[0]  = UART::BUFFER::START_BYTE;
  _tx_buffer[1]  = UART::COMMAND::DISABLE;
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);
}

RobotStatusReport IotShield::getStatusReport()
{
  return _report;
}

void IotShield::processStatusReport(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer)
{
  _report.temperature = rxBufferToTemperature(buffer);
  _report.voltage.mcu = rxBufferToVoltage(buffer);
  _report.current.mcu = rxBufferToCurrent(buffer);
}

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
