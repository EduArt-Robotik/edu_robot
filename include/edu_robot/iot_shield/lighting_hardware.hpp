/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/lighting.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class LightingHardware : public Lighting::ComponentInterface
                       , public eduart::robot::iotbot::IotShieldTxDevice
{
public:
  LightingHardware(const std::string& hardware_name, std::shared_ptr<IotShieldCommunicator> communicator);
  ~LightingHardware() override;

  void processSetValue(const Color& color, const robot::Lighting::Mode& mode) override;
  void initialize(const Lighting::Parameter& parameter) override;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
