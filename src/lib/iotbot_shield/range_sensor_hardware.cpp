#include "edu_robot/iot_shield/range_sensor_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"

#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

RangeSensorHardware::RangeSensorHardware(const std::string& hardware_name, const std::uint8_t id, 
                                         const RangeSensor::Parameter& parameter)
  : IotShieldDevice(hardware_name)
  , IotShieldRxDevice(hardware_name)
  , _id(id)
{
  (void)parameter;
}

void RangeSensorHardware::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  uart::message::ShieldResponse msg(data);

  switch (_id) {
  case 0u:
    _callback_process_measurement(msg.range0());
    break;

  case 1u:
    _callback_process_measurement(msg.range1());
    break;

  case 2u:
    _callback_process_measurement(msg.range2());
    break;

  case 3u:
    _callback_process_measurement(msg.range3());
    break;

  default:
    throw std::runtime_error("Invalid id! Iot Shield can have ids 0 .. 3 only.");
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot