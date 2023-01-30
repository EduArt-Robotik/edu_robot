#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;
using tcp::message::SetLighting;
using tcp::message::PROTOCOL;

LightingHardware::LightingHardware(const std::string& hardware_name,
                                   std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayTxDevice(hardware_name, communicator) 
{

}

LightingHardware::~LightingHardware()
{

}

void LightingHardware::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;

  // HACK! At the moment each light can't controlled separately.
  switch (mode) {
  case Mode::FLASH:
    if (name().find("left") != std::string::npos) {
      auto request = Request::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::LEFT, color.r, color.g, color.b);
      auto response = _communicator->sendRequest(std::move(request));
      wait_for_future(response, 100ms);
      response.get();
    }
    else if (name().find("right") != std::string::npos) {
      auto request = Request::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::RIGHT, color.r, color.g, color.b);
      auto response = _communicator->sendRequest(std::move(request));
      wait_for_future(response, 100ms);
      response.get();
    }
    else if (name().find("all") != std::string::npos) {
      auto request = Request::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::ALL, color.r, color.g, color.b);
      auto response = _communicator->sendRequest(std::move(request));
      wait_for_future(response, 100ms);
      response.get();
    }
    break;

    // all lightings are addressed
  case Mode::DIM: {
    auto request = Request::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::DIM_LIGHT, color.r, color.g, color.b);
    auto response = _communicator->sendRequest(std::move(request));
    wait_for_future(response, 100ms);
    response.get();
  }
  break;

  case Mode::OFF: {
    auto request = Request::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::LIGHTS_OFF, color.r, color.g, color.b);
    auto response = _communicator->sendRequest(std::move(request));
    wait_for_future(response, 100ms);
    response.get();    
  }
  break;

  case Mode::PULSATION: {
    auto request = Request::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::PULSATION, color.r, color.g, color.b);
    auto response = _communicator->sendRequest(std::move(request));
    wait_for_future(response, 100ms);
    response.get();    
  }  
  break;

  case Mode::ROTATION: {
    auto request = Request::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::ROTATION, color.r, color.g, color.b);
    auto response = _communicator->sendRequest(std::move(request));
    wait_for_future(response, 100ms);
    response.get();
  }
  break;

  case Mode::RUNNING: {
    auto request = Request::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::RUNNING, color.r, color.g, color.b);
    auto response = _communicator->sendRequest(std::move(request));
    wait_for_future(response, 100ms);
    response.get();    
  }
  break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }
}

void LightingHardware::initialize(const Lighting::Parameter& parameter)
{
  (void)parameter;
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
