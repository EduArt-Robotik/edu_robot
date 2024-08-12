#include "edu_robot/hardware/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/communicator_node.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/protocol.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"
#include <chrono>
#include <functional>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

using namespace std::chrono_literals;

using udp::message::GetImuMeasurement;
using udp::message::SetImuParameter;
using udp::message::Acknowledgement;
using udp::message::AcknowledgedImuMeasurement;

using udp::message::PROTOCOL;

ImuSensorHardware::ImuSensorHardware(std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  _communication_node->addSendingJob(
    std::bind(&ImuSensorHardware::processSending, this), 100ms
  );
}

void ImuSensorHardware::initialize(const SensorImu::Parameter& parameter)
{
  auto request = EthernetRequest::make_request<SetImuParameter>(
    parameter.raw_data_mode,
    parameter.fusion_weight,
    parameter.mount_orientation.roll,
    parameter.mount_orientation.pitch,
    parameter.mount_orientation.yaw
  );
  const auto got = _communication_node->sendRequest(std::move(request), 100ms);

  if (Acknowledgement<PROTOCOL::COMMAND::SET::IMU_PARAMETER>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Motor Controller Parameter\" was not acknowledged.");
  }
}

void ImuSensorHardware::processSending()
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  try {
    // Get measurement data from ethernet gateway and parse it to processing pipeline.
    auto request = EthernetRequest::make_request<GetImuMeasurement>();
    const auto got = _communication_node->sendRequest(std::move(request), 200ms);

    _callback_process_measurement(
      AcknowledgedImuMeasurement::orientation(got.response()),
      AcknowledgedImuMeasurement::angularVelocity(got.response()),
      AcknowledgedImuMeasurement::linearAcceleration(got.response())
    );
  }
  catch (...) {
    // \todo handle error case!
  }
}


} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
