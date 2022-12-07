/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/range_sensor.hpp>
#include <edu_robot/hardware_component_interface.hpp>
#include <rclcpp/parameter.hpp>

#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

class RangeSensorHardware : public HardwareSensorInterface<float>
                          , public EthernetGatewayRxDevice
{
public:
  struct Parameter {

  };

  RangeSensorHardware(const std::string& hardware_name, const std::uint8_t id, const Parameter& parameter);
  ~RangeSensorHardware() override = default;

  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;

private:
  Parameter _parameter;
  std::uint8_t _id;
};               

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
