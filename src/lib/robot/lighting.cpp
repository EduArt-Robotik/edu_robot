#include "edu_robot/lighting.hpp"
#include <memory>

namespace eduart {
namespace robot {

Lighting::Lighting(const std::string& name, const Color default_color, const float default_brightness,
                   std::shared_ptr<ComponentInterface> hardware_interface)
  : _name(name)
  , _hardware_interface(std::move(hardware_interface))
{
  (void)default_color;
  (void)default_brightness;
}

void Lighting::setColor(const Color color, const Mode mode)
{
  _hardware_interface->processSetValue(color, mode);
  _color = color;
}

void Lighting::setBrightness(const float brightness)
{
  // processSetBrightness(brightness);
  // \todo implement method!
  _brightness = brightness;
}

} // end namespace eduart
} // end namespace robot
