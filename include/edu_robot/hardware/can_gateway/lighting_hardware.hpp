/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/lighting.hpp>

#include <memory>
#include <map>
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
 * \brief These classes here provide a functionality to address specific lightings like front, rear, left and right.
 */
class LightingGroup : public Lighting::ComponentInterface
{
public:
  LightingGroup(const std::string& name) : _name(name) { }
  ~LightingGroup() override = default;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override {
    (void)parameter;
  }

private:
  std::string _name;
};

class LightingHardwareManager
{
private:
  friend std::unique_ptr<LightingHardwareManager> std::make_unique<LightingHardwareManager>();

  LightingHardwareManager();

public:
  friend LightingGroup;

  ~LightingHardwareManager() = default;

  static LightingHardwareManager& instance() {
    if (_instance == nullptr) {
      _instance = std::make_unique<LightingHardwareManager>();
    }

    return *_instance;
  }

  /**
   * \brief Initialize via the sensor ring MeasurementManager.
   *
   * \param manager                The shared MeasurementManager that owns the sensor ring lights.
   * \param zone_to_light_indices  Maps zone names ("all", "head", …) to light board indices in manager->lights().
   */
  void initialize(
    std::shared_ptr<sensorring::manager::MeasurementManager> manager,
    const std::unordered_map<std::string, std::vector<int>>& zone_to_light_indices);

  inline std::shared_ptr<Lighting::ComponentInterface> lighting(const std::string& name) {
    return _lighting_group.at(name);
  }

private:
  void processSetValue(const std::string& name, const Color& color, const robot::Lighting::Mode& mode);

  inline static std::unique_ptr<LightingHardwareManager> _instance{nullptr};

  std::shared_ptr<sensorring::manager::MeasurementManager> _manager;
  std::unordered_map<std::string, std::vector<int>> _zone_to_light_indices;

  std::map<std::string, std::shared_ptr<LightingGroup>> _lighting_group;
};



} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
