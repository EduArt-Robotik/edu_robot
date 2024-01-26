/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware/igus/can_communicator.hpp"
#include "edu_robot/hardware/igus/can_gateway_device_interfaces.hpp"
#include "edu_robot/rpm.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace igus {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public CanGatewayTxRxDevice
{
public:
  MotorControllerHardware(
    const std::string& name, const std::uint8_t can_id, std::shared_ptr<CanCommunicator> communicator)
    : MotorController::HardwareInterface(name, 1)
    , CanGatewayTxRxDevice(communicator)
    , _can_id(can_id)
    , _measured_rpm(1, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void processRxData(const can::message::RxMessageDataBuffer& data) override;

private:
  std::uint8_t _can_id;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
