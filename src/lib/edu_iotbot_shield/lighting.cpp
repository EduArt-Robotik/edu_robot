#include "edu_robot/iot_shield/lighting.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/iot_shield/uart/uart_message_conversion.hpp"
#include "edu_robot/lighting.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

using uart::message::UART;

Lighting::Lighting(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
                   const Color default_color, const float default_brightness)
  : IotShieldDevice(name, id)
  , eduart::robot::Lighting(name, default_color, default_brightness)
  , IotShieldTxDevice(name, id, communicator) 
{

}

Lighting::~Lighting()
{

}

bool Lighting::processSetColor(const Color color, const Mode mode)
{
  // clear buffer
  _tx_buffer = { 0 };

  // HACK! At the moment each light can't controlled separately.
  switch (mode) {
  case Mode::FLASH:
    if (robot::Lighting::name().find("left") != std::string::npos) {
      _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::LEFT>(color).data()); 
    }
    else if (robot::Lighting::name().find("right") != std::string::npos) {
      _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::RIGHT>(color).data());
    }
    else if (robot::Lighting::name().find("all") != std::string::npos) {
      _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::ALL>(color).data());
    }
    break;

    // all lightings are addressed
  case Mode::DIM:
    _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::DIM>(color).data());
    break;

  case Mode::OFF:
    _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::OFF>(color).data());
    break;

  case Mode::PULSATION:
    _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::PULSATION>(color).data());
    break;

  case Mode::ROTATION:
    _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::ROTATION>(color).data());
    break;

  case Mode::RUNNING:
    _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::RUNNING>(color).data());
    break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }

  return true;
}

bool Lighting::processSetBrightness(const float brightness)
{
  // \todo no idea how to set the brightness at the moment
  return true;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
