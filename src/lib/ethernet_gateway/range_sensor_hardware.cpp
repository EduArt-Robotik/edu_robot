#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

using tcp::message::GetDistanceMeasurement;
using tcp::message::AcknowledgedDistanceMeasurement;

RangeSensorHardware::RangeSensorHardware(
  const std::string& hardware_name, const std::uint8_t id, rclcpp::Node& ros_node,
  std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayTxRxDevice(hardware_name, communicator)
  , _id(id)
  , _timer_get_measurement(
      ros_node.create_wall_timer(100ms, std::bind(&RangeSensorHardware::processMeasurement, this))
    )
{

}

void RangeSensorHardware::processRxData(const tcp::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(AcknowledgedDistanceMeasurement::distance(data));
}

void RangeSensorHardware::initialize(const RangeSensor::Parameter& parameter)
{
  (void)parameter;
}

void RangeSensorHardware::processMeasurement()
{
  try {
    // Get measurement data from ethernet gateway and parse it to processing pipeline.
    auto request = Request::make_request<GetDistanceMeasurement>(_id);
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    processRxData(future_response.get().response());  
  }
  catch (...) {
    // \todo handle error case!
  }
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
