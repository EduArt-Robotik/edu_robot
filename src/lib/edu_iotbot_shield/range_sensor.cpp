#include "edu_robot/iot_shield/range_sensor.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

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