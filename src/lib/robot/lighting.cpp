#include "edu_robot/lighting.hpp"
#include <memory>

namespace eduart {
namespace robot {

Lighting::Lighting(const std::string& name, const Color default_color, const float default_brightness,
                   std::shared_ptr<ComponentInterface> hardware_interface)
  : _color(default_color)
  , _brightness(default_brightness)
  , _name(name)
  , _hardware_interface(std::move(hardware_interface))
{

}

void Lighting::setColor(const Color color, const Mode mode)
{
  _mode = mode;
  _color = color;
  _hardware_interface->processSetValue(_color * _brightness, _mode);
}

void Lighting::setBrightness(const float brightness)
{
  // processSetBrightness(brightness);
  // \todo implement method!
  _brightness = brightness;
  _hardware_interface->processSetValue(_color * brightness, _mode);
}

} // end namespace eduart
} // end namespace robot
