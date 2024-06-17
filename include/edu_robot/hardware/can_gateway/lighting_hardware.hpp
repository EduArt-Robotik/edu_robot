/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/lighting.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
  
class LightingHardware : public Lighting::ComponentInterface
{
public:
  LightingHardware(
    const std::string& name, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator);
  ~LightingHardware() override;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override;

private:
  void processSending();

  std::string _name;
  std::shared_ptr<CommunicatorNode> _communication_node;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
