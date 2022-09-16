#include "edu_robot/iot_shield/lighting.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart_message_conversion.hpp"
#include "edu_robot/lighting.hpp"
#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

Lighting::Lighting(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
                   const Color default_color, const float default_brightness)
  : eduart::robot::Lighting(name, default_color, default_brightness)
  , IotShieldDevice(name, id, communicator) 
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
  std::uint8_t command = 0x00;

  switch (mode) {
  case Mode::FLASH:
    if (robot::Lighting::name().find("left") != std::string::npos) {
      command = UART::COMMAND::LIGHTING::FLASH::LEFT;
    }
    else if (robot::Lighting::name().find("right") != std::string::npos) {
      command = UART::COMMAND::LIGHTING::FLASH::RIGHT;
    }
    else if (robot::Lighting::name().find("all") != std::string::npos) {
      command = UART::COMMAND::LIGHTING::FLASH::ALL;
    }
    break;

    // all lightings are addressed
  case Mode::DIM:
    command = UART::COMMAND::LIGHTING::DIM;
    break;

  case Mode::OFF:
    command = UART::COMMAND::LIGHTING::OFF;
    break;

  case Mode::PULSATION:
    command = UART::COMMAND::LIGHTING::PULSATION;
    break;

  case Mode::ROTATION:
    command = UART::COMMAND::LIGHTING::ROTATION;
    break;

  case Mode::RUNNING:
    command = UART::COMMAND::LIGHTING::RUNNING;
    break;

  default:
    throw std::invalid_argument("given mode is not handled");
  }


  // prepare message and send it
  _tx_buffer[ 0] = UART::BUFFER::START_BYTE;
  _tx_buffer[ 1] = command;
  _tx_buffer[ 2] = color.r;
  _tx_buffer[ 3] = color.g;
  _tx_buffer[ 4] = color.b;
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);

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
