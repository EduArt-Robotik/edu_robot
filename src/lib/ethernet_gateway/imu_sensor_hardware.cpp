#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

using tcp::message::GetImuMeasurement;
using tcp::message::AcknowledgedImuMeasurement;

ImuSensorHardware::ImuSensorHardware(
  const std::string& hardware_name, rclcpp::Node& ros_node, std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayTxRxDevice(hardware_name, communicator)
  , _timer_get_measurement(
      ros_node.create_wall_timer(100ms, std::bind(&ImuSensorHardware::processMeasurement, this))
    )
{

}
 
void ImuSensorHardware::processRxData(const tcp::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  _callback_process_measurement(AcknowledgedImuMeasurement::orientation(data));
}

void ImuSensorHardware::initialize(const ImuSensor::Parameter& parameter)
{
  (void)parameter;
  // set IMU data mode
  // auto request = ShieldRequest::make_request<tcp::message::SetImuRawDataMode>(parameter.raw_data_mode, 0, 0, 0);
  // auto response = _communicator->sendRequest(std::move(request));
  // wait_for_future(response, 100ms);
  // response.get();  
}

void ImuSensorHardware::processMeasurement()
{
  // Get measurement data from ethernet gateway and parse it to processing pipeline.
  auto request = Request::make_request<GetImuMeasurement>();
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);

  processRxData(future_response.get().response());
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
