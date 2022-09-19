#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/iot_shield/uart/uart_message_conversion.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <memory>
#include <functional>
#include <iostream>

namespace eduart {
namespace robot {
namespace iotbot {

using uart::message::UART;

IotShield::IotShield(char const* const device_name)
  : _communicator(std::make_shared<IotShieldCommunicator>(device_name))
{
  _communicator->registerProcessReceivedBytes(
    std::bind(&IotShield::processStatusReport, this, std::placeholders::_1)
  );

  // set UART timeout
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::UART_TIMEOUT>(1.0f).data());
  // and IMU data mode to raw
  _communicator->sendBytes(uart::message::SetImuRawDataMode(false).data());
}

IotShield::~IotShield()
{
  // \todo do some clean up on hardware side!
}

void IotShield::enable()
{
  _communicator->sendBytes(uart::message::Enable().data());
}

void IotShield::disable()
{
  _communicator->sendBytes(uart::message::Disable().data());
}

RobotStatusReport IotShield::getStatusReport()
{
  return _report;
}

void IotShield::processStatusReport(const std::array<std::uint8_t, uart::message::UART::BUFFER::RX_SIZE>& buffer)
{
  uart::message::ShieldResponse<UART::COMMAND::SET::RPM>msg(buffer);
  std::cout << "message valid = " << msg.isMessageCandidateValid() << std::endl;
  _report.temperature = static_cast<float>(msg.getElementValue<6>()) / 100.0f;// uart::message::rxBufferToTemperature(buffer);
  _report.voltage.mcu = static_cast<float>(msg.getElementValue<9>()) / 100.0f;// std::size_t Index>()  uart::message::rxBufferToVoltage(buffer);
  _report.current.mcu = static_cast<float>(msg.getElementValue<10>()) / 20.0f;// uart::message::rxBufferToCurrent(buffer);
  _status_report_ready = true;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
