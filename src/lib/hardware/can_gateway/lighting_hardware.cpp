#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

LightingHardware::LightingHardware(const std::string& name, std::shared_ptr<Communicator> communicator)
  : CommunicatorTxDevice(communicator)
  , _name(name)
{

}

LightingHardware::~LightingHardware()
{

}

void LightingHardware::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;

  // HACK! At the moment each light can't controlled separately.
  // switch (mode) {
  // case Mode::FLASH:
  //   if (_name.find("left") != std::string::npos) {
  //     auto request = EthernetRequest::make_request<SetLighting>(
  //       PROTOCOL::LIGHTING::MODE::FLASH::LEFT, color.r, color.g, color.b);
  //     auto response = _communicator->sendRequest(std::move(request));
  //     wait_for_future(response, 100ms);
  //     response.get();
  //   }
  //   else if (_name.find("right") != std::string::npos) {
  //     auto request = EthernetRequest::make_request<SetLighting>(
  //       PROTOCOL::LIGHTING::MODE::FLASH::RIGHT, color.r, color.g, color.b);
  //     auto response = _communicator->sendRequest(std::move(request));
  //     wait_for_future(response, 100ms);
  //     response.get();
  //   }
  //   else if (_name.find("all") != std::string::npos) {
  //     auto request = EthernetRequest::make_request<SetLighting>(
  //       PROTOCOL::LIGHTING::MODE::FLASH::ALL, color.r, color.g, color.b);
  //     auto response = _communicator->sendRequest(std::move(request));
  //     wait_for_future(response, 100ms);
  //     response.get();
  //   }
  //   break;

  //   // all lightings are addressed
  // case Mode::DIM: {
  //   auto request = EthernetRequest::make_request<SetLighting>(
  //     PROTOCOL::LIGHTING::MODE::DIM_LIGHT, color.r, color.g, color.b);
  //   auto response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(response, 100ms);
  //   response.get();
  // }
  // break;

  // case Mode::OFF: {
  //   auto request = EthernetRequest::make_request<SetLighting>(
  //     PROTOCOL::LIGHTING::MODE::LIGHTS_OFF, color.r, color.g, color.b);
  //   auto response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(response, 100ms);
  //   response.get();    
  // }
  // break;

  // case Mode::PULSATION: {
  //   auto request = EthernetRequest::make_request<SetLighting>(
  //     PROTOCOL::LIGHTING::MODE::PULSATION, color.r, color.g, color.b);
  //   auto response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(response, 100ms);
  //   response.get();    
  // }  
  // break;

  // case Mode::ROTATION: {
  //   auto request = EthernetRequest::make_request<SetLighting>(
  //     PROTOCOL::LIGHTING::MODE::ROTATION, color.r, color.g, color.b);
  //   auto response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(response, 100ms);
  //   response.get();
  // }
  // break;

  // case Mode::RUNNING: {
  //   auto request = EthernetRequest::make_request<SetLighting>(
  //     PROTOCOL::LIGHTING::MODE::RUNNING, color.r, color.g, color.b);
  //   auto response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(response, 100ms);
  //   response.get();    
  // }
  // break;

  // default:
  //   throw std::invalid_argument("given mode is not handled");
  // }
}

void LightingHardware::initialize(const Lighting::Parameter& parameter)
{
  (void)parameter;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
