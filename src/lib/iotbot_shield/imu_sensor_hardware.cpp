#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

ImuSensorHardware::ImuSensorHardware(const std::string& hardware_name, const ImuSensor::Parameter parameter,
                                     std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name)
  , IotShieldTxRxDevice(hardware_name, communicator)
{
  // set IMU data mode
  _communicator->sendBytes(uart::message::SetImuRawDataMode(parameter.raw_mode).data());
}
 
void ImuSensorHardware::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  uart::message::ShieldResponse msg(data);

  _callback_process_measurement(msg.imuOrientation());
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
