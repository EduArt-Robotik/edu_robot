/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/color.hpp"
#include "edu_robot/hardware_component_interface.hpp"

#include <memory>
#include <string>

namespace eduart {
namespace robot {

/**
 * \brief Represents a RGB lighting without any hardware relation, providing lighting functionality.
 *        This class needs to be realized by a specific hardware layer.
 *
 * \todo below is only a draft, a first try --> please review concept after first release
 */
class Lighting
{
public:
  // \todo mode do not really fit to a single lighting.
  enum class Mode {
    OFF,
    DIM,
    FLASH,
    PULSATION,
    ROTATION,
    RUNNING,
  };

  struct Parameter {

  };

  using ComponentInterface = HardwareComponentInterface<Parameter, Color, Mode>;

  Lighting(const std::string& name, const Color default_color, const float default_brightness,
           std::shared_ptr<ComponentInterface> hardware_interface);
  virtual ~Lighting() = default;

  /**
   * \brief Sets the color of this lighting.
   *
   * \param color The new color of the lighting.
   * \param mode The new mode of the lighting.
   * \throw HardwareError
   * \return true if the brightness was set successfully.
   */
  void setColor(const Color color, const Mode mode);
  /**
   * \brief Sets the brightness of this lighting.
   *
   * \param brightness The new brightness. Must be in range of 0 .. 1.
   * \throw HardwareError
   * \return true if the brightness was set successfully.
   */
  void setBrightness(const float brightness);

  inline const std::string& name() const { return _name; }

private:
  Color _color;
  float _brightness;
  std::string _name;
  std::shared_ptr<ComponentInterface> _hardware_interface;
};

} // end namespace eduart
} // end namespace robot
