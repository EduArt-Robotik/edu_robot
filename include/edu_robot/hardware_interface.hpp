/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {

/**
 * \brief Interface used to have a base for all hardware interfaces classes.
 */
class HardwareInterface : public std::enable_shared_from_this<HardwareInterface>
{
public:
  enum class Type {
    UNKOWN,
    MOTOR_CONTROLLER,
    LIGHTING,
    SENOR_IMU,
    SENSOR_RANGE,
    SENSOR_POINT_CLOUD,
  };

  HardwareInterface(const Type type) : _type(type) { }
  virtual ~HardwareInterface() = default;

  inline Type type() const { return  _type; }

  template <class Type>
  std::shared_ptr<Type> cast() {
    auto hardware = std::dynamic_pointer_cast<Type>(shared_from_this());

    if (hardware == nullptr) {
      std::invalid_argument("HardwareInterface::cast(): invalid cast.");
    }
    // else: type cast is valid

    return hardware;
  }

private:
  Type _type;
};

} // end namespace robot
} // end namespace eduart
