/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/lighting.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class LightingHardware : public Lighting::ComponentInterface
                       , public eduart::robot::ethernet::EthernetGatewayTxDevice
{
public:
  LightingHardware(const std::string& name, std::shared_ptr<EthernetCommunicator> communicator);
  ~LightingHardware() override;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override;

private:
  std::string _name;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
