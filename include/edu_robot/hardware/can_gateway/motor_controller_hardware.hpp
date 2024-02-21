/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware/communicator_device_interfaces.hpp"
#include "edu_robot/rpm.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public CommunicatorTxRxDevice
{
public:
  struct Parameter {
    struct {
      std::uint32_t input;
      std::uint32_t output;
    } can_id;
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, 2)
    , CommunicatorTxRxDevice(communicator)
    , _parameter(parameter)
    , _measured_rpm(2, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void processRxData(const message::RxMessageDataBuffer& data) override;
  void enable();
  void disable();

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  const Parameter _parameter;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
