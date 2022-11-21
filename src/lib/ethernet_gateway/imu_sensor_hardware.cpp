#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

ImuSensorHardware::ImuSensorHardware(const std::string& hardware_name, const ImuSensor::Parameter parameter,
                                     std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name)
  , EthernetGatewayTxRxDevice(hardware_name, communicator)
{
  // set IMU data mode
  // auto request = ShieldRequest::make_request<tcp::message::SetImuRawDataMode>(parameter.raw_data_mode, 0, 0, 0);
  // auto response = _communicator->sendRequest(std::move(request));
  // wait_for_future(response, 100ms);
  // response.get();
}
 
void ImuSensorHardware::processRxData(const tcp::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  // _callback_process_measurement(uart::message::ShieldResponse::imuOrientation(data));
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
