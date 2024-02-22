#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using can::CanRxDataEndPoint;
using can::message::sensor::imu::Response;

ImuSensorHardware::ImuSensorHardware(rclcpp::Node& ros_node, const std::uint32_t can_id, std::shared_ptr<Communicator> communicator)
  : CommunicatorTxRxDevice(communicator)
  , _can_id(can_id)
{
  (void)ros_node;

  auto data_endpoint = CanRxDataEndPoint::make_data_endpoint<Response>(
    can_id,
    std::bind(&ImuSensorHardware::processRxData, this, std::placeholders::_1),
    this
  );
  _communicator->registerRxDataEndpoint(data_endpoint);
  registerRxDataEndpoint(data_endpoint);
}
 
void ImuSensorHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  _callback_process_measurement(
    Response::orientation(data), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
  );
}

void ImuSensorHardware::initialize(const SensorImu::Parameter& parameter)
{
  (void)parameter;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
