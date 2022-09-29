#include "edu_robot/iot_shield/range_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/range_sensor.hpp"
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

RangeSensor::RangeSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
                         const tf2::Transform sensor_transform, const std::uint8_t id, const Parameter parameter,
                         rclcpp::Node& ros_node)
  : IotShieldDevice(name, id)
  , eduart::robot::RangeSensor(name, frame_id, reference_frame_id, sensor_transform, parameter, ros_node)
  , IotShieldRxDevice(name, id)
{

}

void RangeSensor::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  uart::message::ShieldResponse msg(data);

  switch (id()) {
  case 0u:
    processMeasurementData(msg.range0());
    break;

  case 1u:
    processMeasurementData(msg.range1());
    break;

  case 2u:
    processMeasurementData(msg.range2());
    break;

  case 3u:
    processMeasurementData(msg.range3());
    break;

  default:
    throw std::runtime_error("Invalid id! Iot Shield can have ids 0 .. 3 only.");
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot