#include "edu_robot/hardware/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"

#include <rclcpp/qos.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

using namespace std::chrono_literals;

using udp::message::GetDistanceMeasurement;
using udp::message::AcknowledgedDistanceMeasurement;

RangeSensorHardware::RangeSensorHardware(
  const std::uint8_t id, rclcpp::Node& ros_node, std::shared_ptr<Communicator> communicator)
  : CommunicatorTxRxNode(communicator)
  , _id(id)
  , _timer_get_measurement(
      ros_node.create_wall_timer(100ms, std::bind(&RangeSensorHardware::processMeasurement, this))
    )
{

}

void RangeSensorHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(AcknowledgedDistanceMeasurement::distance(data));
}

void RangeSensorHardware::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void RangeSensorHardware::processMeasurement()
{
  try {
    // Get measurement data from ethernet gateway and parse it to processing pipeline.
    auto request = EthernetRequest::make_request<GetDistanceMeasurement>(_id);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    processRxData(future_response.get().response());
  }
  catch (...) {
    // \todo handle error case!
  }
}

void RangeSensorHardware::doCommunication()
{
  
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
