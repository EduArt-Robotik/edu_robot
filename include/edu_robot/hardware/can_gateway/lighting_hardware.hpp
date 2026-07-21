/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/lighting.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declaration for sensorring type
namespace eduart {
namespace sensorring {
namespace manager { class MeasurementManager; }
} // end namespace sensorring
} // end namespace eduart

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class LightingHardwareManager;

/**
 * \brief Lighting zone backed by the sensorring library.
 *
 * Each instance represents one logical zone (e.g. "head", "left_side") and
 * delegates light commands to the shared LightingHardwareManager.
 */
class LightingGroup : public Lighting::ComponentInterface
{
public:
  LightingGroup(const std::string& name, std::shared_ptr<LightingHardwareManager> manager);
  ~LightingGroup() override = default;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override {
    (void)parameter;
  }

private:
  std::string _name;
  std::shared_ptr<LightingHardwareManager> _manager;
};

/**
 * \brief Manages sensorring light devices and maps logical lighting zones to physical light indices.
 *
 * Created by HardwareComponentFactory::addLighting() after the sensor ring is set up.
 */
class LightingHardwareManager : public std::enable_shared_from_this<LightingHardwareManager>
{
public:
  /**
   * \brief Construct a fully-initialized lighting manager.
   *
   * \param manager                The MeasurementManager that owns the light devices.
   * \param zone_to_light_indices  Maps zone names ("all", "head", …) to light indices in manager->lights().
   */
  LightingHardwareManager(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    std::unordered_map<std::string, std::vector<std::size_t>> zone_to_light_indices);
  ~LightingHardwareManager() = default;

  void processSetValue(const std::string& zone_name, const Color& color, const robot::Lighting::Mode& mode);

private:
  std::shared_ptr<sensorring::manager::MeasurementManager> _manager;
  std::unordered_map<std::string, std::vector<std::size_t>> _zone_to_light_indices;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
