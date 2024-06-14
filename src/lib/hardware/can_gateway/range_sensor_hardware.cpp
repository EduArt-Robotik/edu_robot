#include "edu_robot/hardware/can_gateway/range_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"

#include <rclcpp/qos.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

RangeSensorHardware::RangeSensorHardware(
  const std::uint8_t id, std::shared_ptr<Communicator> communicator)
  : CommunicatorTxRxNode(communicator)
  , _id(id)
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

void RangeSensorHardware::doCommunication()
{
  
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
