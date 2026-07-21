#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"

#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/device/light/LightMode.hpp>

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

LightingGroup::LightingGroup(const std::string& name, std::shared_ptr<LightingHardwareManager> manager)
  : _name(name)
  , _manager(std::move(manager))
{ }

void LightingGroup::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  _manager->processSetValue(_name, color, mode);
}

LightingHardwareManager::LightingHardwareManager(
  std::shared_ptr<sensorring::manager::MeasurementManager> manager,
  std::unordered_map<std::string, std::vector<std::size_t>> zone_to_light_indices)
  : _manager(std::move(manager))
  , _zone_to_light_indices(std::move(zone_to_light_indices))
{ }

void LightingHardwareManager::processSetValue(
  const std::string& zone_name, const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;
  using sensorring::device::LightMode;

  LightMode light_mode = LightMode::Off;
  switch (mode) {
  case Mode::OFF:
    light_mode = LightMode::Off;
    break;
  case Mode::DIM:
    light_mode = LightMode::FixedColor;
    break;
  case Mode::FLASH:
    if (zone_name == "left_side") {
      light_mode = LightMode::FlashLeft;
    } else if (zone_name == "right_side") {
      light_mode = LightMode::FlashRight;
    } else {
      light_mode = LightMode::FlashAll;
    }
    break;
  case Mode::PULSATION:
    light_mode = LightMode::PulsationColor;
    break;
  case Mode::ROTATION:
    light_mode = LightMode::Rotation;
    break;
  case Mode::RUNNING:
    light_mode = LightMode::Running;
    break;
  default:
    throw std::invalid_argument("LightingHardwareManager: given mode is not handled");
  }

  const auto zone_it = _zone_to_light_indices.find(zone_name);
  if (zone_it == _zone_to_light_indices.end()) {
    throw std::invalid_argument("LightingHardwareManager: unknown zone '" + zone_name + "'");
  }

  auto lights = _manager->lights();
  for (const std::size_t idx : zone_it->second) {
    lights[idx].setLight(light_mode, color.r, color.g, color.b);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
