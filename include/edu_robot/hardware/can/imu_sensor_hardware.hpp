/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/hardware/can/can_gateway_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public CanGatewayTxRxDevice
{
public:
  ImuSensorHardware(rclcpp::Node& ros_node, const std::uint32_t can_id, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const message::RxMessageDataBuffer& data) override;
  void initialize(const SensorImu::Parameter& parameter) override;

private:
  std::uint32_t _can_id;
};

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
