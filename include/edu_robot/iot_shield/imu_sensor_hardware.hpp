/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/imu_sensor.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/iotbot.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class ImuSensorHardware : public robot::HardwareSensorInterface<Eigen::Quaterniond>
                        , public IotShieldTxRxDevice
{
public:
  ImuSensorHardware(const std::string& hardware_name, const ImuSensor::Parameter parameter,
            std::shared_ptr<IotShieldCommunicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
