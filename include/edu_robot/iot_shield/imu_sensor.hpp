/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iotbot.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class ImuSensor : public eduart::robot::ImuSensor
                , public IotShieldTxRxDevice
{
public:
  ImuSensor(const std::string& name, const std::string& frame_id, const tf2::Transform sensor_transform,
            const std::uint8_t id, const Parameter parameter, std::shared_ptr<IotShieldCommunicator> communicator,
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, std::shared_ptr<rclcpp::Node> ros_node);
  ~ImuSensor() override = default;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
