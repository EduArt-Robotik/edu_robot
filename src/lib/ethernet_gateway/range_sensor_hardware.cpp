#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace ethernet {

RangeSensorHardware::RangeSensorHardware(const std::string& hardware_name, const std::uint8_t id)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayRxDevice(hardware_name)
  , _id(id)
{

}

void RangeSensorHardware::processRxData(const tcp::message::RxMessageDataBuffer& data)
{
  (void)data;

  if (_callback_process_measurement == nullptr) {
    return;
  }

  // \todo implement data processing!
}

void RangeSensorHardware::initialize(const RangeSensor::Parameter& parameter)
{
  (void)parameter;
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
