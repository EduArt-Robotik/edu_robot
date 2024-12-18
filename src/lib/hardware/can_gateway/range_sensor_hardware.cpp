#include "edu_robot/hardware/can_gateway/range_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <memory>
#include <rclcpp/qos.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;
using can::CanRxDataEndPoint;

RangeSensorHardware::RangeSensorHardware(
  const std::uint8_t id, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _id(id)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  
}

void RangeSensorHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  (void)data;

  if (_callback_process_measurement == nullptr) {
    return;
  }

  // _callback_process_measurement(AcknowledgedDistanceMeasurement::distance(data));
}

void RangeSensorHardware::initialize(const SensorRange::Parameter& parameter)
{
  (void)parameter;
}

void RangeSensorHardware::processMeasurement()
{
  // try {
  //   // Get measurement data from ethernet gateway and parse it to processing pipeline.
  //   auto request = EthernetRequest::make_request<GetDistanceMeasurement>(_id);
  //   auto future_response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(future_response, 100ms);

  //   processRxData(future_response.get().response());  
  // }
  // catch (...) {
  //   // \todo handle error case!
  // }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
