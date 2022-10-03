/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/range_sensor.hpp>
#include <edu_robot/hardware_component_interface.hpp>

#include "edu_robot/iot_shield/iot_shield_device.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class RangeSensorHardware : public HardwareSensorInterface<float>
                          , public IotShieldRxDevice
{
public:
  RangeSensorHardware(const std::string& hardware_name, const std::uint8_t id, const RangeSensor::Parameter& parameter);
  ~RangeSensorHardware() override = default;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;

private:
  std::uint8_t _id;
};               

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
