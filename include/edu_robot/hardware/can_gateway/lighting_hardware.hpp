/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstdint>
#include <edu_robot/lighting.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

#include <memory>
#include <map>

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

  struct Parameter {
    std::uint32_t can_address = 0x200;
  };

  ~LightingHardwareManager() = default;

  static LightingHardwareManager& instance() {
    if (_instance == nullptr) {
      _instance = std::make_unique<LightingHardwareManager>();
    }

    return *_instance;

    // alternative implementation
    // static LightingHardwareManager manager;
    // return manager;
  }

  void initialize(
    std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator_left, std::shared_ptr<Communicator> communicator_right);
  inline std::shared_ptr<Lighting::ComponentInterface> lighting(const std::string& name) {
    return _lighting_group.at(name);
  }

private:
  void processSetValue(const std::string& name, const Color& color, const robot::Lighting::Mode& mode);
  void syncLighting();

  inline static std::unique_ptr<LightingHardwareManager> _instance{nullptr};

  const Parameter _parameter;
  std::shared_ptr<CommunicatorNode> _communication_node_left;
  std::shared_ptr<CommunicatorNode> _communication_node_right;
  std::map<std::string, std::shared_ptr<LightingGroup>> _lighting_group;
};



} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
