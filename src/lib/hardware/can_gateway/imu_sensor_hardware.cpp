#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"

#include <edu_robot/hardware/communicator_node.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using can::Request;
using can::CanRxDataEndPoint;
using can::message::sensor::imu::MeasurementOrientation;
using can::message::sensor::imu::MeasurementRaw;
using can::message::sensor::imu::SetImuFusion;
using can::message::sensor::imu::SetImuOrientation;

ImuSensorHardware::ImuSensorHardware(
  const std::uint32_t can_id_host, const std::uint32_t can_id_board, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator)
  : _can_id_host(can_id_host)
  , _can_id_board(can_id_board)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
{
  _processing_data.clear();

  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, MeasurementOrientation>(
    can_id_host,
    std::bind(&ImuSensorHardware::processRxData, this, std::placeholders::_1)
  );
}
 
void ImuSensorHardware::processRxData(const message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }

  // Orientation Measurement
  if (MeasurementOrientation::hasCorrectLength(data)) {
    _processing_data.gotOrientation();
    _processing_data.orientation = MeasurementOrientation::orientation(data);
  }
  // Raw Measurements
  else if (MeasurementRaw::hasCorrectLength(data)) {
    // Linear Acceleration
    if (MeasurementRaw::isLinearAcceleration(data)) {
      _processing_data.linear_acceleration = MeasurementRaw::linearAcceleration(data);
      _processing_data.gotLinearAcceleration();
    }
    // Angular Velocity
    else if (MeasurementRaw::isAngularVelocity(data)) {
      _processing_data.angular_velocity = MeasurementRaw::angularVelocity(data);
      _processing_data.gotAngularVelocity();
    }
  }

  // If complete parse it to sensor instance.
  if (_processing_data.complete() == false) {
    return;
  }

  _callback_process_measurement(
    _processing_data.orientation, _processing_data.angular_velocity, _processing_data.linear_acceleration
  );
  _processing_data.clear();
}

void ImuSensorHardware::initialize(const SensorImu::Parameter& parameter)
{
  // setting imu fusion
  {
    auto request = Request::make_request<SetImuFusion>(_can_id_board, parameter.raw_data_mode);
    _communication_node->sendRequest(std::move(request), 100ms);
  }
  // setting imu orientation
  {
    auto request = Request::make_request<SetImuOrientation>(
      _can_id_board,
      parameter.mount_orientation.roll,
      parameter.mount_orientation.pitch,
      parameter.mount_orientation.yaw
    );
    _communication_node->sendRequest(std::move(request), 100ms);
  }
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
