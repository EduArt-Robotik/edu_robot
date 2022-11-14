#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

#include <cstddef>
#include <exception>
#include <iterator>
#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

EthernetGatewayShield::EthernetGatewayShield(char const* const ip_address, const std::uint16_t port)
  : _communicator(std::make_shared<EthernetCommunicator>(ip_address, port))
{
  for (std::size_t i = 0; i < 1000; ++i) {
    std::cout << "start new iteration" << std::endl;
    try {
      auto request = Request::make_request<tcp::message::GetFirmwareVersion>();
      auto future_response = _communicator->sendRequest(std::move(request));
      // wait_for_future(future_response, 100ms);

      auto got = future_response.get();

      std::cout << "size rx buffer = " << got.response().size() << std::endl;
      for (const auto byte : got.response()) {
        std::cout << std::hex << static_cast<int>(byte) << " ";
      }
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
