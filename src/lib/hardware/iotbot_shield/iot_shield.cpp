#include "edu_robot/hardware/iot_shield/iot_shield.hpp"
#include "edu_robot/hardware/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/hardware/iot_shield/uart/message.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"

#include <edu_robot/robot_status_report.hpp>

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
  , _communicator(std::make_shared<IotShieldCommunicator>(device_name)) 
{
  // Configuring Diagnostic
  _clock = std::make_shared<rclcpp::Clock>();
  _diagnostic.voltage = std::make_shared<diagnostic::MeanDiagnostic<float, std::less<float>>>(
    "voltage", "V", 10, 18.7f, 18.0f
  );
  _diagnostic.current = std::make_shared<diagnostic::MeanDiagnostic<float, std::greater<float>>>(
    "current", "A", 10, 1.5f, 2.0f
  );
  _diagnostic.temperature = std::make_shared<diagnostic::MeanDiagnostic<float, std::greater<float>>>(
    "temperature", "°C", 10, 55.0f, 65.0f
  );
  _diagnostic.processing_dt = std::make_shared<diagnostic::StandardDeviationDiagnostic<std::uint64_t, std::greater<std::uint64_t>>>(
    "processing dt", "ms", 10, 700, 1000, 50, 100
  );
  _diagnostic.last_processing = _clock->now();

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
  // processStatusReport();
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
  // Do Status Report
  const bool need_to_update = 
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - _communicator->getStampRxBuffer()) > 1500ms; // \todo make configurable!

  if (need_to_update) {
    auto request = ShieldRequest::make_request<uart::message::SetRpm>(0.0f, 0.0f, 0.0f, 0.0f);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    future_response.get();    
  }

  if (_communicator->isRxBufferNew() == false) {
    return;
  }

  const auto buffer = _communicator->getRxBuffer();
  _report.voltage.mcu = uart::message::ShieldResponse::voltage(buffer);
  _report.current.mcu = uart::message::ShieldResponse::current(buffer);
  _report.rpm.resize(4u);
  _report.rpm[0] = -uart::message::ShieldResponse::rpm0(buffer);
  _report.rpm[1] = -uart::message::ShieldResponse::rpm1(buffer);
  _report.rpm[2] = -uart::message::ShieldResponse::rpm2(buffer);
  _report.rpm[3] = -uart::message::ShieldResponse::rpm3(buffer);

  if (_imu_raw_data_mode) {
    _report.temperature = std::numeric_limits<float>::quiet_NaN();
  }
  else {
    _report.temperature = uart::message::ShieldResponse::temperature(buffer);
  }

  sendInputValue(_report.voltage.mcu);

  for (auto& device : _rx_devices) {
    device->processRxData(buffer);
  }

  // Do Diagnostics
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();
  _diagnostic.processing_dt->update(dt / 1000000);
  _diagnostic.last_processing = now;

  _diagnostic.voltage->update(_report.voltage.mcu);
  _diagnostic.current->update(_report.current.mcu);
  _diagnostic.temperature->update(_report.temperature);
}

diagnostic::Diagnostic IotShield::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;

  diagnostic.add(*_diagnostic.voltage);
  diagnostic.add(*_diagnostic.current);
  diagnostic.add(*_diagnostic.temperature);
  diagnostic.add(*_diagnostic.processing_dt);

  return diagnostic;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
