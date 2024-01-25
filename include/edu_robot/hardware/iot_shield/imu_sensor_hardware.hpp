/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/hardware/iot_shield/iot_shield_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public IotShieldTxRxDevice
{
public:
  ImuSensorHardware(std::shared_ptr<IotShieldCommunicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
  void initialize(const SensorImu::Parameter& parameter) override;

private:
  bool _raw_mode = true;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
