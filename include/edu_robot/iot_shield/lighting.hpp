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
namespace iot_bot {

class Lighting : public eduart::robot::Lighting
               , public eduart::robot::iot_bot::IotShieldDevice
{
public:
  Lighting(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
           const Color default_color, const float default_brightness);
  ~Lighting() override;

  void setColor(const Color color) override;
  void setBrightness(const float brightness) override;

private:
  // \todo do some iot hardware things here...
};

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
