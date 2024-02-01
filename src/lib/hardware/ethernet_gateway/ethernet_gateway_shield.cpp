#include "edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_communication_device.hpp"

#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/protocol.hpp"

#include <cstddef>
#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

using namespace std::chrono_literals;
using udp::message::PROTOCOL;
using udp::message::Acknowledgement;
using udp::message::SetEncoderParameter;
using udp::message::SetMotorControllerParameter;
using udp::message::SetPidControllerParameter;
using udp::message::AcknowledgedStatus;

EthernetGatewayShield::EthernetGatewayShield(char const* const ip_address, const std::uint16_t port)
  : processing::ProcessingComponentOutput<float>("ethernet_gateway_shield")
  , _communicator(std::make_shared<Communicator>(std::make_shared<EthernetCommunicationDevice>(ip_address, port)))
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

  // Performing Firmware Check
  auto request = EthernetRequest::make_request<udp::message::GetFirmwareVersion>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);

  struct {
    std::uint8_t major;
    std::uint8_t minor;
    std::uint8_t patch;
  } firmware_version;

  auto got = future_response.get();
  firmware_version.major = got.response().data()[3];
  firmware_version.minor = got.response().data()[4];
  firmware_version.patch = got.response().data()[5];

  if (firmware_version.minor < 2 && firmware_version.patch < 1) {
    throw std::runtime_error("Ethernet Gateway's firmware is not compatible.");
  }

  std::cout << std::dec << std::endl;

  std::cout << static_cast<int>(got.response().data()[3]) << '.'
            << static_cast<int>(got.response().data()[4]) << '.'
            << static_cast<int>(got.response().data()[5]) << ": "
            << &got.response().data()[6]
            << std::endl << std::endl << std::endl;
}

EthernetGatewayShield::~EthernetGatewayShield()
{
  disable();
}

void EthernetGatewayShield::enable()
{
  for (std::size_t i = 0; i < 2; ++i) {
    auto request = EthernetRequest::make_request<udp::message::SetMotorEnabled>();
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_ENABLE>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Motor Enabled\" was not acknowledged.");
    }
  }
}

void EthernetGatewayShield::disable()
{
  auto request = EthernetRequest::make_request<udp::message::SetMotorDisabled>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);

  auto got = future_response.get();
  if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_DISABLE>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Motor Enabled\" was not acknowledged.");
  }
}

RobotStatusReport EthernetGatewayShield::getStatusReport()
{
  // Requesting Status Report
  auto request = EthernetRequest::make_request<udp::message::GetStatus>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();

  // Do Status Report
  RobotStatusReport report;

  report.temperature = AcknowledgedStatus::temperature(got.response());
  report.voltage.mcu = AcknowledgedStatus::voltage(got.response());
  report.current.mcu = AcknowledgedStatus::current(got.response());
  report.status_emergency_stop = AcknowledgedStatus::statusEmergencyButton(got.response());

  sendInputValue(report.voltage.mcu);

  // Do Diagnostics
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();
  _diagnostic.processing_dt->update(dt / 1000000);
  _diagnostic.last_processing = now;

  _diagnostic.voltage->update(report.voltage.mcu);
  _diagnostic.current->update(report.current.mcu);
  _diagnostic.temperature->update(report.temperature);

  return report;
}

diagnostic::Diagnostic EthernetGatewayShield::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;

  diagnostic.add(*_diagnostic.voltage);
  diagnostic.add(*_diagnostic.current);
  diagnostic.add(*_diagnostic.temperature);
  diagnostic.add(*_diagnostic.processing_dt);

  return diagnostic;
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
