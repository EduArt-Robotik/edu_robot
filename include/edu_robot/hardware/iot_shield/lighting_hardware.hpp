/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/lighting.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class LightingHardware : public Lighting::ComponentInterface
                       , public CommunicatorTxNode
{
public:
  LightingHardware(const std::string& name, std::shared_ptr<Communicator> communicator);
  ~LightingHardware() override;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override;

private:
  void doCommunication() override;

  std::string _name;  
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
