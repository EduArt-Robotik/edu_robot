#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

ImuSensorHardware::ImuSensorHardware(const std::string& hardware_name, std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name)
  , IotShieldTxRxDevice(hardware_name, communicator)
{

}

void ImuSensorHardware::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  _callback_process_measurement(
    uart::message::ShieldResponse::imuOrientation(data), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
  );
}

void ImuSensorHardware::initialize(const ImuSensor::Parameter& parameter)
{
  // set IMU data mode
  auto request = ShieldRequest::make_request<uart::message::SetImuRawDataMode>(parameter.raw_data_mode, 0, 0, 0);
  auto response = _communicator->sendRequest(std::move(request));
  wait_for_future(response, 100ms);
  response.get();
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
