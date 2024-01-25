/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware_component_interfaces.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device_interfaces.hpp"
#include "edu_robot/rpm.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public eduart::robot::iotbot::IotShieldTxRxDevice
{
public:
  MotorControllerHardware(const std::string& name, std::shared_ptr<IotShieldCommunicator> communicator);
  ~MotorControllerHardware() override;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;

private:
  std::vector<Rpm> _measured_rpm;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
