/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/lighting.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class Lighting : public eduart::robot::Lighting
               , public eduart::robot::iotbot::IotShieldTxDevice
{
public:
  Lighting(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
           const Color default_color, const float default_brightness);
  ~Lighting() override;

  bool processSetColor(const Color color, const Mode mode) override;
  bool processSetBrightness(const float brightness) override;

private:
  // \todo do some iot hardware things here...
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
