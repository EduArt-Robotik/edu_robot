#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

using tcp::message::GetImuMeasurement;
using tcp::message::SetImuParameter;
using tcp::message::Acknowledgement;
using tcp::message::AcknowledgedImuMeasurement;

using tcp::message::PROTOCOL;

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
  
  _callback_process_measurement(
    AcknowledgedImuMeasurement::orientation(data),
    AcknowledgedImuMeasurement::angularVelocity(data),
    AcknowledgedImuMeasurement::linearAcceleration(data)
  );
}

void ImuSensorHardware::initialize(const ImuSensor::Parameter& parameter)
{
  auto request = Request::make_request<SetImuParameter>(
    parameter.raw_data_mode,
    parameter.fusion_weight,
    parameter.mount_orientation.roll,
    parameter.mount_orientation.pitch,
    parameter.mount_orientation.yaw
  );
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);

  auto got = future_response.get();
  if (Acknowledgement<PROTOCOL::COMMAND::SET::IMU_PARAMETER>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Motor Controller Parameter\" was not acknowledged.");
  } 
}

void ImuSensorHardware::processMeasurement()
{
  try {
    // Get measurement data from ethernet gateway and parse it to processing pipeline.
    auto request = Request::make_request<GetImuMeasurement>();
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
