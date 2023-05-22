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
using namespace std::chrono_literals;

IotShield::IotShield(char const* const device_name)
  : processing::ProcessingComponentOutput<float>("iot_shield")
  ,_communicator(std::make_shared<IotShieldCommunicator>(device_name)) 
{
  // set UART timeout
  auto request = ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::UART_TIMEOUT>>(
    1.0f, 0);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  future_response.get();
}

IotShield::~IotShield()
{
  // \todo do some clean up on hardware side!
}

void IotShield::enable()
{
  auto request = ShieldRequest::make_request<uart::message::Enable>(0, 0);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  future_response.get();  
}

void IotShield::disable()
{
  auto request = ShieldRequest::make_request<uart::message::Disable>(0, 0);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  future_response.get();    
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

void IotShield::processStatusReport()
{
  const auto buffer = _communicator->getRxBuffer();
  _report.temperature = uart::message::ShieldResponse::temperature(buffer);
  _report.voltage.mcu = uart::message::ShieldResponse::voltage(buffer);
  _report.current.mcu = uart::message::ShieldResponse::current(buffer);
  _report.rpm.resize(4u);
  _report.rpm[0] = -uart::message::ShieldResponse::rpm0(buffer);
  _report.rpm[1] = -uart::message::ShieldResponse::rpm1(buffer);
  _report.rpm[2] = -uart::message::ShieldResponse::rpm2(buffer);
  _report.rpm[3] = -uart::message::ShieldResponse::rpm3(buffer);
  
  _status_report_ready = true;
  sendInputValue(_report.voltage.mcu);

  for (auto& device : _rx_devices) {
    device->processRxData(buffer);
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
