#include "edu_robot/hardware/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/hardware/communicator_node.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"

#include <memory>
#include <rclcpp/qos.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

using namespace std::chrono_literals;

using udp::message::GetDistanceMeasurement;
using udp::message::AcknowledgedDistanceMeasurement;

RangeSensorHardware::RangeSensorHardware(
  const std::uint8_t id, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _id(id)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  _communication_node->addSendingJob(
    std::bind(&RangeSensorHardware::processSending, this), 100ms
  );
}

void RangeSensorHardware::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void RangeSensorHardware::processSending()
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  try {
    // Get measurement data from ethernet gateway and parse it to processing pipeline.
    auto request = EthernetRequest::make_request<GetDistanceMeasurement>(_id);
    const auto got = _communication_node->sendRequest(std::move(request), 100ms);

    _callback_process_measurement(AcknowledgedDistanceMeasurement::distance(got.response()));
  }
  catch (...) {
    // \todo handle error case!
  }
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
