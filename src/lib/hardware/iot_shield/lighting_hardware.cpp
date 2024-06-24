#include "edu_robot/hardware/iot_shield/lighting_hardware.hpp"
#include "edu_robot/hardware/iot_shield/uart/message.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/iot_shield/uart/uart_request.hpp>

#include <edu_robot/lighting.hpp>

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using uart::message::UART;
using uart::Request;

using namespace std::chrono_literals;

LightingHardware::LightingHardware(
  const std::string& name, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _name(name)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
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
    if (_name.find("left") != std::string::npos) {
      auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::LEFT>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);
    }
    else if (_name.find("right") != std::string::npos) {
      auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::RIGHT>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);
    }
    else if (_name.find("all") != std::string::npos) {
      auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::ALL>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);
    }
    break;

    // all lightings are addressed
  case Mode::DIM: {
    auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::DIM>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);
  }
  break;

  case Mode::OFF: {
    auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::OFF>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);   
  }
  break;

  case Mode::PULSATION: {
    auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::PULSATION>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);   
  }  
  break;

  case Mode::ROTATION: {
    auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::ROTATION>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);
  }
  break;

  case Mode::RUNNING: {
    auto request = Request::make_request<uart::message::SetLighting<UART::COMMAND::LIGHTING::RUNNING>>(
      color.r, color.g, color.b, 0, 0);
      _communication_node->sendRequest(std::move(request), 100ms);  
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

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
