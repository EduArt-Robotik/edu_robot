/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/color.hpp"

#include <string>

namespace eduart {
namespace robot {

/**
 * \brief Represents a RGB lighting without any hardware relation, providing lighting functionality.
 *        This class needs to be realized by a specific hardware layer.
 */
class Lighting
{
protected:
  Lighting(const std::string& name, const Color default_color, const float default_brightness)
    : _name(name)
  {
    setColor(default_color);
    setBrightness(default_brightness);
  }

public:
  virtual ~Lighting() = default;

  /**
   * \brief Sets the color of this lighting.
   *
   * \param color The new color.
   * \throw HardwareError
   * \return true if the brightness was set successfully.
   */
  bool virtual setColor(const Color color);
  /**
   * \brief Sets the brightness of this lighting.
   *
   * \param brightness The new brightness. Must be in range of 0 .. 1.
   * \throw HardwareError
   * \return true if the brightness was set successfully.
   */
  bool virtual setBrightness(const float brightness);

  const std::string& name() const { return _name; }

protected:
  Color _color;
  float _brightness;

private:
  std::string _name;
};

} // end namespace eduart
} // end namespace robot
