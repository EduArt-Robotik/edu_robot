/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/range_sensor.hpp>
#include <edu_robot/hardware_component_interface.hpp>

#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

class RangeSensorHardware : public RangeSensor::SensorInterface
                          , public EthernetGatewayTxRxDevice
{
public:
  RangeSensorHardware(
    const std::string& hardware_name, const std::uint8_t id, rclcpp::Node& ros_node,
    std::shared_ptr<EthernetCommunicator> communicator);
  ~RangeSensorHardware() override = default;

  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;
  void initialize(const RangeSensor::Parameter& parameter) override;

private:
  void processMeasurement();

  std::uint8_t _id;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
};               

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
