/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/hardware/communicator_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public CommunicatorTxRxDevice
{
public:
  ImuSensorHardware(rclcpp::Node& ros_node, const std::uint32_t can_id, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processRxData(const message::RxMessageDataBuffer& data);

  std::uint32_t _can_id;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
