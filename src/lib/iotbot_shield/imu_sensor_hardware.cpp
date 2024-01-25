#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/sensor_imu.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

ImuSensorHardware::ImuSensorHardware(std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldTxRxDevice(communicator)
{

}

void ImuSensorHardware::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  if (_raw_mode == false) {
    _callback_process_measurement(
      uart::message::ShieldResponse::imuOrientation(data), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
    );
  }
  else {
    _callback_process_measurement(
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
      uart::message::ShieldResponse::angularVelocity(data),
      uart::message::ShieldResponse::linearAcceleration(data)
    );
  }
}

void ImuSensorHardware::initialize(const SensorImu::Parameter& parameter)
{
  // set IMU data mode
  auto request = ShieldRequest::make_request<uart::message::SetImuRawDataMode>(parameter.raw_data_mode, 0, 0, 0);
  auto response = _communicator->sendRequest(std::move(request));
  wait_for_future(response, 100ms);
  response.get();

  _raw_mode = parameter.raw_data_mode;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
