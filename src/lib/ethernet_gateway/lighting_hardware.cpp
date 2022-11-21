#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

#include <stdexcept>
#include <string>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

LightingHardware::LightingHardware(const std::string& hardware_name,
                                   std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayTxDevice(hardware_name, communicator) 
{

}

LightingHardware::~LightingHardware()
{

}

void LightingHardware::processSetValue(const Color& color, const robot::Lighting::Mode& mode)
{
  (void)color;
  (void)mode;
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
