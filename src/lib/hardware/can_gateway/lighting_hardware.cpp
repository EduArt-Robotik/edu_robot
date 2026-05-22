#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/device/light/LightMode.hpp>

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

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
  std::shared_ptr<sensorring::manager::MeasurementManager> manager,
  const std::unordered_map<std::string, std::vector<int>>& zone_to_light_indices)
{
  _manager = std::move(manager);
  _zone_to_light_indices = zone_to_light_indices;
}

void LightingHardwareManager::processSetValue(const std::string& name, const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;
  using sensorring::device::LightMode;

  LightMode light_mode = LightMode::Off;
  switch (mode) {
  case Mode::OFF:        light_mode = LightMode::Off;       break;
  case Mode::DIM:        light_mode = LightMode::Dimmed;    break;
  case Mode::FLASH:
    if (name == "left_side") {
      light_mode = LightMode::FlashLeft;
    } else if (name == "right_side") {
      light_mode = LightMode::FlashRight;
    } else {
      light_mode = LightMode::FlashAll;
    }
    break;
  case Mode::PULSATION:  light_mode = LightMode::Pulsation; break;
  case Mode::ROTATION:   light_mode = LightMode::Rotation;  break;
  case Mode::RUNNING:    light_mode = LightMode::Running;   break;
  default:
    throw std::invalid_argument("LightingHardwareManager: given mode is not handled");
  }

  const auto zone_it = _zone_to_light_indices.find(name);
  if (zone_it == _zone_to_light_indices.end()) {
    throw std::invalid_argument("LightingHardwareManager: unknown zone '" + name + "'");
  }

  auto lights = _manager->lights();
  for (const int idx : zone_it->second) {
    lights[static_cast<std::size_t>(idx)].setMode(light_mode);
    lights[static_cast<std::size_t>(idx)].setColor(color.r, color.g, color.b);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
