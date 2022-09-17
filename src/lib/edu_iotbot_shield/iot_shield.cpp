#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart_message_conversion.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <memory>
#include <functional>

namespace eduart {
namespace robot {
namespace iotbot {

IotShield::IotShield(char const* const device_name)
  : _communicator(std::make_shared<IotShieldCommunicator>(device_name))
{
  _communicator->registerProcessReceivedBytes(
    std::bind(&IotShield::processStatusReport, this, std::placeholders::_1)
  );

  // set UART timeout
  _tx_buffer = { 0 };

  _tx_buffer[0] = UART::BUFFER::START_BYTE;
  _tx_buffer[1] = UART::COMMAND::SET::UART_TIMEOUT;
  floatToTxBuffer<2, 5>(1.0f, _tx_buffer);
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);
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

  // HACK BEGIN
  _tx_buffer = { 0 };

  _tx_buffer[0] = UART::BUFFER::START_BYTE;
  _tx_buffer[1] = UART::COMMAND::SET::IMU_RAW_DATA;
  _tx_buffer[2] = 0x00;
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);
  // HACK END
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
  _status_report_ready = true;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
