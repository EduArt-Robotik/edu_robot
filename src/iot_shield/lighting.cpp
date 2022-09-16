#include "edu_robot/iot_shield/lighting.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"

namespace eduart {
namespace robot {
namespace iot_bot {

Lighting::Lighting(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
                   const Color default_color, const float default_brightness)
  : eduart::robot::Lighting(name, default_color, default_brightness)
  , IotShieldDevice(name, id, communicator) 
{

}

void Lighting::setColor(const Color color)
{
  // clear buffer
  _tx_buffer = { 0 };

  // prepare message and send it
  _tx_buffer[ 0] = UART::BUFFER::START_BYTE;
  _tx_buffer[ 1] = id();
  _tx_buffer[ 2] = color.r;
  _tx_buffer[ 3] = color.g;
  _tx_buffer[ 4] = color.b;
  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);
}

void Lighting::setBrightness(const float brightness)
{
  // \todo no idea how to set the brightness at the moment
}

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
