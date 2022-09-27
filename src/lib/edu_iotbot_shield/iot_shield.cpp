#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <algorithm>
#include <memory>
#include <functional>
#include <iostream>
#include <stdexcept>

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

void IotShield::registerIotShieldRxDevice(std::shared_ptr<IotShieldRxDevice> device)
{
  if (std::find(_rx_devices.begin(), _rx_devices.end(), device) != _rx_devices.end()) {
    throw std::invalid_argument("Given IotShieldRxDevice is already contained in rx device container.");
  }

  _rx_devices.push_back(device);
}

void IotShield::processStatusReport(const uart::message::RxMessageDataBuffer& buffer)
{
  uart::message::ShieldResponse msg(buffer);

  _report.temperature = msg.temperature();
  _report.voltage.mcu = msg.voltage();
  _report.current.mcu = msg.current();
  _report.rpm.resize(4u);
  _report.rpm[0] = msg.rpm0();
  _report.rpm[1] = msg.rpm1();
  _report.rpm[2] = msg.rpm2();
  _report.rpm[3] = msg.rpm3();
  
  _status_report_ready = true;

  for (auto& device : _rx_devices) {
    device->processRxData(buffer);
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
