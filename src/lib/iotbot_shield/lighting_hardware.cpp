#include "edu_robot/iot_shield/lighting_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/lighting.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

using uart::message::UART;
using namespace std::chrono_literals;

LightingHardware::LightingHardware(const std::string& hardware_name,
                                   std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name)
  , IotShieldTxDevice(hardware_name, communicator) 
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
  // case Mode::FLASH:
  //   if (robot::Lighting::name().find("left") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::LEFT>(color).data()); 
  //   }
  //   else if (robot::Lighting::name().find("right") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::RIGHT>(color).data());
  //   }
  //   else if (robot::Lighting::name().find("all") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::ALL>(color).data());
  //   }
  //   break;

    // all lightings are addressed
  case Mode::DIM: {
    auto request = ShieldRequest::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::DIM>>(
      color.r, color.g, color.b, 0, 0);
    auto response = _communicator->sendRequest(std::move(request));
    response.wait_for(100ms);
    response.get();
  }
  break;

  case Mode::OFF: {
    auto request = ShieldRequest::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::OFF>>(
      color.r, color.g, color.b, 0, 0);
    auto response = _communicator->sendRequest(std::move(request));
    response.wait_for(100ms);
    response.get();    
  }
  break;

  case Mode::PULSATION: {
    auto request = ShieldRequest::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::PULSATION>>(
      color.r, color.g, color.b, 0, 0);
    auto response = _communicator->sendRequest(std::move(request));
    response.wait_for(100ms);
    response.get();    
  }  
  break;

  case Mode::ROTATION: {
    auto request = ShieldRequest::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::ROTATION>>(
      color.r, color.g, color.b, 0, 0);
    auto response = _communicator->sendRequest(std::move(request));
    response.wait_for(100ms);
    response.get();
  }
  break;

  case Mode::RUNNING: {
    auto request = ShieldRequest::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::RUNNING>>(
      color.r, color.g, color.b, 0, 0);
    auto response = _communicator->sendRequest(std::move(request));
    response.wait_for(100ms);
    response.get();    
  }
  break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
