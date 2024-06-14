#include "edu_robot/hardware/iot_shield/range_sensor_hardware.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"

#include <edu_robot/sensor_range.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

RangeSensorHardware::RangeSensorHardware(const std::uint8_t id, std::shared_ptr<Communicator> communicator)
  : CommunicatorRxNode(communicator)
  , _id(id)
{

}

void RangeSensorHardware::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  switch (_id) {
  case 0u:
    _callback_process_measurement(uart::message::ShieldResponse::range0(data));
    break;

  case 1u:
    _callback_process_measurement(uart::message::ShieldResponse::range1(data));
    break;

  case 2u:
    _callback_process_measurement(uart::message::ShieldResponse::range2(data));
    break;

  case 3u:
    _callback_process_measurement(uart::message::ShieldResponse::range3(data));
    break;

  default:
    throw std::runtime_error("Invalid id! Iot Shield can have ids 0 .. 3 only.");
  }
}

void RangeSensorHardware::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
