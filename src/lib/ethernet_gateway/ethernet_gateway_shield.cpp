#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"

#include <cstddef>
#include <exception>
#include <iterator>
#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;
using tcp::message::PROTOCOL;
using tcp::message::Acknowledgement;
using tcp::message::SetEncoderParameter;
using tcp::message::SetMotorControllerParameter;
using tcp::message::SetPidControllerParameter;
using tcp::message::AcknowledgedStatus;

EthernetGatewayShield::EthernetGatewayShield(char const* const ip_address, const std::uint16_t port)
  : processing::ProcessingComponentOutput<float>("ethernet_gateway_shield")
  , _communicator(std::make_shared<EthernetCommunicator>(ip_address, port))
{
  auto request = Request::make_request<tcp::message::GetFirmwareVersion>();
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
  auto request = Request::make_request<tcp::message::SetMotorEnabled>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);

  auto got = future_response.get();
  if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_ENABLE>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Motor Enabled\" was not acknowledged.");
  }
}

void EthernetGatewayShield::disable()
{
  auto request = Request::make_request<tcp::message::SetMotorDisabled>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);

  auto got = future_response.get();
  if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_DISABLE>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Motor Enabled\" was not acknowledged.");
  }
}

RobotStatusReport EthernetGatewayShield::getStatusReport()
{
  auto request = Request::make_request<tcp::message::GetStatus>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);

  auto got = future_response.get();
  RobotStatusReport report;

  report.temperature = AcknowledgedStatus::temperature(got.response());
  report.voltage.mcu = AcknowledgedStatus::voltage(got.response());
  report.current.mcu = AcknowledgedStatus::current(got.response());
  report.status_emergency_stop = AcknowledgedStatus::statusEmergencyButton(got.response());  

  sendInputValue(report.voltage.mcu);

  return report;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
