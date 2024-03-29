/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/hardware/ethernet_gateway/ethernet_gateway_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public EthernetGatewayTxRxDevice
{
public:
  ImuSensorHardware(rclcpp::Node& ros_node, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const message::RxMessageDataBuffer& data) override;
  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processMeasurement();

  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
