#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using can::CanRxDataEndPoint;
using can::message::sensor::imu::MeasurementOrientation;
using can::message::sensor::imu::MeasurementRaw;

ImuSensorHardware::ImuSensorHardware(const std::uint32_t can_id, std::shared_ptr<Communicator> communicator)
  : CommunicatorTxRxDevice(communicator)
  , _can_id(can_id)
{
  _processing_data.clear();

  auto data_endpoint = createRxDataEndPoint<CanRxDataEndPoint, MeasurementOrientation>(
    can_id,
    std::bind(&ImuSensorHardware::processRxData, this, std::placeholders::_1)
  );
  _communicator->registerRxDataEndpoint(data_endpoint);
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
  (void)parameter;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
