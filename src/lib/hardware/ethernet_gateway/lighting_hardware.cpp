#include "edu_robot/hardware/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/communicator_node.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/protocol.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

using namespace std::chrono_literals;
using udp::message::SetLighting;
using udp::message::PROTOCOL;

LightingHardware::LightingHardware(
  const std::string& name, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _name(name)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  _communication_node->addSendingJob(
    std::bind(&LightingHardware::processSending, this), 100ms
  );
}

LightingHardware::~LightingHardware()
{

}

void LightingHardware::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  std::scoped_lock lock(_data.mutex);
  _data.color = color;
  _data.mode = mode;
}

void LightingHardware::initialize(const Lighting::Parameter& parameter)
{
  (void)parameter;
}

void LightingHardware::processSending()
{
  using Mode = robot::Lighting::Mode;
  std::scoped_lock lock(_data.mutex);

  // HACK! At the moment each light can't controlled separately.
  switch (_data.mode) {
  case Mode::FLASH:
    if (_name.find("left") != std::string::npos) {
      auto request = EthernetRequest::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::LEFT, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
    }
    else if (_name.find("right") != std::string::npos) {
      auto request = EthernetRequest::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::RIGHT, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
    }
    else if (_name.find("all") != std::string::npos) {
      auto request = EthernetRequest::make_request<SetLighting>(
        PROTOCOL::LIGHTING::MODE::FLASH::ALL, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
    }
    break;

    // all lightings are addressed
  case Mode::DIM: {
    auto request = EthernetRequest::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::DIM_LIGHT, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
  }
  break;

  case Mode::OFF: {
    auto request = EthernetRequest::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::LIGHTS_OFF, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);  
  }
  break;

  case Mode::PULSATION: {
    auto request = EthernetRequest::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::PULSATION, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);  
  }  
  break;

  case Mode::ROTATION: {
    auto request = EthernetRequest::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::ROTATION, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
  }
  break;

  case Mode::RUNNING: {
    auto request = EthernetRequest::make_request<SetLighting>(
      PROTOCOL::LIGHTING::MODE::RUNNING, _data.color.r, _data.color.g, _data.color.b);
      _communication_node->sendRequest(std::move(request), 200ms);
  }
  break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
