#include "edu_robot/iot_shield/imu_sensor.hpp"
#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

ImuSensor::ImuSensor(const std::string& name, const std::string& frame_id, const tf2::Transform sensor_transform,
                     const std::uint8_t id, const Parameter parameter, std::shared_ptr<IotShieldCommunicator> communicator,
                     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, std::shared_ptr<rclcpp::Node> ros_node)
  : IotShieldDevice(name, id)
  , eduart::robot::ImuSensor(name, frame_id, sensor_transform, parameter, tf_broadcaster, ros_node)
  , IotShieldTxRxDevice(name, id, communicator)
{
  // and IMU data mode to raw
  _communicator->sendBytes(uart::message::SetImuRawDataMode(parameter.raw_mode).data());
};

void ImuSensor::processRxData(const uart::message::RxMessageDataBuffer& data)
{
  uart::message::ShieldResponse msg(data);

  processMeasurementData(msg.imuOrientation());
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
