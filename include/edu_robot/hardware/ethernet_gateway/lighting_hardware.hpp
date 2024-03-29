/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/lighting.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_gateway_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {
  
class LightingHardware : public Lighting::ComponentInterface
                       , public EthernetGatewayTxDevice
{
public:
  LightingHardware(const std::string& name, std::shared_ptr<Communicator> communicator);
  ~LightingHardware() override;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override;

private:
  std::string _name;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
