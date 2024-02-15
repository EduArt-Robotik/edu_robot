/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware/can/can_gateway_device_interfaces.hpp"
#include "edu_robot/rpm.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public CanGatewayTxRxDevice
{
public:
  MotorControllerHardware(
    const std::string& name, const std::uint32_t can_id_input, const std::uint32_t can_id_output,
    std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, 2)
    , CanGatewayTxRxDevice(communicator)
    , _can_id{can_id_input, can_id_output}
    , _measured_rpm(2, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void processRxData(const message::RxMessageDataBuffer& data) override;

private:
  struct {
    std::uint32_t input;
    std::uint32_t output;
  } _can_id;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
