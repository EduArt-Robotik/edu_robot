/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public EthernetGatewayTxRxDevice
{
public:
  ImuSensorHardware(
    const std::string& hardware_name, rclcpp::Node& ros_node, std::shared_ptr<EthernetCommunicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;
  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processMeasurement();

  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
