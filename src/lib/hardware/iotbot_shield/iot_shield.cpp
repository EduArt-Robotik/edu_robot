#include "edu_robot/hardware/iot_shield/iot_shield.hpp"
#include "edu_robot/executer.hpp"
#include "edu_robot/hardware/communicator_node.hpp"
#include "edu_robot/hardware/iot_shield/uart/message.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"
#include "edu_robot/hardware/iot_shield/uart/uart_request.hpp"
#include "edu_robot/hardware/iot_shield/uart_communication_device.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"

#include <edu_robot/hardware/communicator.hpp>

#include <edu_robot/robot_status_report.hpp>

#include <memory>
#include <functional>
#include <mutex>
#include <iostream>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using uart::Request;
using uart::message::ShieldResponse;
using uart::message::UART;

using namespace std::chrono_literals;

IotShield::IotShield(char const* const device_name)
  : processing::ProcessingComponentOutput<float>("iot_shield")
  , _communicator(std::make_shared<Communicator>(std::make_shared<UartCommunicationDevice>(device_name), 8ms))
  , _executer(std::make_shared<Executer>())
  , _communication_node(std::make_shared<CommunicatorNode>(_executer, _communicator))
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
  auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::UART_TIMEOUT>>(1.0f, 0);
  _communication_node->sendRequest(std::move(request), 100ms);

  // create data endpoint for status report
  _communication_node->createRxDataEndPoint<RxDataEndPoint, ShieldResponse>(
    std::bind(&IotShield::processStatusReport, this, std::placeholders::_1)
  );

  // Starting Processing
  _executer->start();
}

IotShield::~IotShield()
{
  disable();
  _executer->stop();
}

void IotShield::enable()
{
  auto request = Request::make_request<uart::message::Enable>(0, 0);
  _communication_node->sendRequest(std::move(request), 100ms);
}

void IotShield::disable()
{
  auto request = Request::make_request<uart::message::Disable>(0, 0);
  _communication_node->sendRequest(std::move(request), 100ms);
}

// is called by main thread
RobotStatusReport IotShield::getStatusReport()
{
  std::lock_guard lock(_data_mutex);  
  return _report;
}

// is called by rx data endpoint thread
void IotShield::processStatusReport(const message::RxMessageDataBuffer& data)
{
  std::lock_guard lock(_data_mutex);

  const auto buffer = data;
  _report.voltage.mcu = ShieldResponse::voltage(buffer);
  _report.current.mcu = ShieldResponse::current(buffer);
  _report.rpm.resize(4u);
  _report.rpm[0] = -ShieldResponse::rpm0(buffer);
  _report.rpm[1] = -ShieldResponse::rpm1(buffer);
  _report.rpm[2] = -ShieldResponse::rpm2(buffer);
  _report.rpm[3] = -ShieldResponse::rpm3(buffer);

  if (_imu_raw_data_mode) {
    _report.temperature = std::numeric_limits<float>::quiet_NaN();
  }
  else {
    _report.temperature = ShieldResponse::temperature(buffer);
  }

  sendInputValue(_report.voltage.mcu);

  // Do Diagnostics
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();
  _diagnostic.processing_dt->update(dt / 1000000);
  _diagnostic.last_processing = now;

  _diagnostic.voltage->update(_report.voltage.mcu);
  _diagnostic.current->update(_report.current.mcu);
  _diagnostic.temperature->update(_report.temperature);
}

// is called by main thread
diagnostic::Diagnostic IotShield::processDiagnosticsImpl()
{
  std::lock_guard lock(_data_mutex);
  diagnostic::Diagnostic diagnostic;

  diagnostic.add(*_diagnostic.voltage);
  diagnostic.add(*_diagnostic.current);
  diagnostic.add(*_diagnostic.temperature);
  diagnostic.add(*_diagnostic.processing_dt);

  return diagnostic;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
