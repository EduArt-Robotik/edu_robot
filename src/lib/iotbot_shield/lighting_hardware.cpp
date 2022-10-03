#include "edu_robot/iot_shield/lighting_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/lighting.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

using uart::message::UART;

LightingHardware::LightingHardware(const std::string& hardware_name,
                                   std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name)
  , IotShieldTxDevice(hardware_name, communicator) 
{

}

LightingHardware::~LightingHardware()
{

}

void LightingHardware::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  using Mode = robot::Lighting::Mode;

  // HACK! At the moment each light can't controlled separately.
  switch (mode) {
  // case Mode::FLASH:
  //   if (robot::Lighting::name().find("left") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::LEFT>(color).data()); 
  //   }
  //   else if (robot::Lighting::name().find("right") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::RIGHT>(color).data());
  //   }
  //   else if (robot::Lighting::name().find("all") != std::string::npos) {
  //     _communicator->sendBytes(uart::message::SetLighting<UART::COMMAND::LIGHTING::FLASH::ALL>(color).data());
  //   }
  //   break;

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
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
