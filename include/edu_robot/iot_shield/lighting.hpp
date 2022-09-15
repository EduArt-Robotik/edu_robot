/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/lighting.hpp"

namespace eduart {
namespace robot {
namespace iot_bot {

class Lighting : public eduart::robot::Lighting
{
public:
  Lighting(const std::string& name);
  ~Lighting() override;

  bool setColor(const Color color) override;
  bool setBrightness(const float brightness) override;

private:
  // \todo do some iot hardware things here...
};

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
