/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/rpm.hpp>

#include <edu_robot/hardware/communicator.hpp>
#include <edu_robot/hardware/can/can_gateway_device_interfaces.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public hardware::can::CanGatewayTxRxDevice
{
public:
  MotorControllerHardware(
    const std::string& name, const std::uint8_t can_id, std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, 1)
    , hardware::can::CanGatewayTxRxDevice(communicator)
    , _can_id(can_id)
    , _measured_rpm(1, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void processRxData(const message::RxMessageDataBuffer& data) override;

private:
  std::uint8_t _can_id;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
