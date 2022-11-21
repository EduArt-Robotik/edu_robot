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

EthernetGatewayShield::EthernetGatewayShield(char const* const ip_address, const std::uint16_t port)
  : _communicator(std::make_shared<EthernetCommunicator>(ip_address, port))
{
  try {
    auto request = Request::make_request<tcp::message::GetFirmwareVersion>();
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();

    std::cout << std::dec << std::endl;

    std::cout << static_cast<int>(got.response().data()[3]) << '.'
              << static_cast<int>(got.response().data()[4]) << '.'
              << static_cast<int>(got.response().data()[5]) << ": "
              << &got.response().data()[6]
              << std::endl << std::endl << std::endl;  
  }
  catch (std::exception& ex) {
    std::cout << "Catch exception: what() = " << ex.what() << std::endl;
  }

  try {
    auto request = Request::make_request<SetMotorControllerParameter>(0, 59.0f, 100.0f, 1.0f, 1.0f, 32);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request was not acknowledged.");
    }
  }
  catch (std::exception& ex) {
    std::cout << "Catch exception: what() = " << ex.what() << std::endl;
  }

  try {
    auto request = Request::make_request<SetEncoderParameter>(0, 2048, 1.0f, true);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request was not acknowledged.");
    }
  }
  catch (std::exception& ex) {
    std::cout << "Catch exception: what() = " << ex.what() << std::endl;
  }

  try {
    auto request = Request::make_request<SetPidControllerParameter>(0, 2.0f, 10.0f, 0.0f, -100.0f, 100.0f, 1.0f, true);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request was not acknowledged.");
    } 
  }
  catch (std::exception& ex) {
    std::cout << "Catch exception: what() = " << ex.what() << std::endl;
  }
}

EthernetGatewayShield::~EthernetGatewayShield()
{

}

void EthernetGatewayShield::enable()
{

}

void EthernetGatewayShield::disable()
{

}

RobotStatusReport EthernetGatewayShield::getStatusReport()
{
  return { };
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
