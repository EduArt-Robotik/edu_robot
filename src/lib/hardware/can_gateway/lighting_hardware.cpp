#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/protocol.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using can::message::lighting::Sync;
using can::message::lighting::SetLighting;
using can::message::PROTOCOL;
using can::Request;

void LightingGroup::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  LightingHardwareManager::instance().processSetValue(_name, color, mode);
}

LightingHardwareManager::LightingHardwareManager()
{
  const std::array<std::string, 5> lighting_name = {"all", "head", "back", "left_side", "right_side"};

  for (const auto& name : lighting_name) {
    _lighting_group[name] = std::make_shared<LightingGroup>(name);
  }
}

void LightingHardwareManager::initialize(
  std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator_left, std::shared_ptr<Communicator> communicator_right)
{
  _communication_node_left  = std::make_shared<CommunicatorNode>(executer, communicator_left);
  _communication_node_right = std::make_shared<CommunicatorNode>(executer, communicator_right);

  syncLighting();
}

void LightingHardwareManager::syncLighting()
{
  // set counter of right side lights to zero
  auto sync_right_side = Request::make_request<Sync>(_parameter.can_address, 0, false);
  _communication_node_right->sendRequest(std::move(sync_right_side), 100ms);

  // set counter of left side lights to zero
  auto sync_left_side = Request::make_request<Sync>(_parameter.can_address, 0, true);
  _communication_node_left->sendRequest(std::move(sync_left_side), 100ms);
}

void LightingHardwareManager::processSetValue(const std::string& name, const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;

  // HACK! At the moment each light can't controlled separately.
  switch (mode) {
  case Mode::FLASH:
    if (name == "left") {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::FLASH_LEFT>>(
        _parameter.can_address, color.r, color.g, color.b);
      _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    else if (name == "right") {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::FLASH_RIGHT>>(
        _parameter.can_address, color.r, color.g, color.b);
      _communication_node_right->sendRequest(std::move(request), 100ms);
    }
    else if (name == "all") {
      // left side
      {
        auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::FLASH_ALL>>(
          _parameter.can_address, color.r, color.g, color.b);
        _communication_node_left->sendRequest(std::move(request), 100ms);
      }
      // right side
      {
        auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::FLASH_ALL>>(
          _parameter.can_address, color.r, color.g, color.b);
        _communication_node_right->sendRequest(std::move(request), 100ms);
      }
    }
    break;

  // all lightings are addressed
  case Mode::DIM: {
    // left side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::DIM_LIGHT>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    // right side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::DIM_LIGHT>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_right->sendRequest(std::move(request), 100ms);
    }    
  }
  break;

  case Mode::OFF: {
    // left side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::LIGHTS_OFF>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    // right side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::LIGHTS_OFF>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_right->sendRequest(std::move(request), 100ms);
    }   
  }
  break;

  case Mode::PULSATION: {
    // left side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::PULSATION>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    // right side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::PULSATION>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_right->sendRequest(std::move(request), 100ms);
    }  
  }  
  break;

  case Mode::ROTATION: {
    // left side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::ROTATION>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    // right side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::ROTATION>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_right->sendRequest(std::move(request), 100ms);
    } 
  }
  break;

  case Mode::RUNNING: {
    // left side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::RUNNING>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_left->sendRequest(std::move(request), 100ms);
    }
    // right side
    {
      auto request = Request::make_request<SetLighting<PROTOCOL::LIGHTING::COMMAND::RUNNING>>(
        _parameter.can_address, color.r, color.g, color.b);
      auto response = _communication_node_right->sendRequest(std::move(request), 100ms);
    }    
  }
  break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
