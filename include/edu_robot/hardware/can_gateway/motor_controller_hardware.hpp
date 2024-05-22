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
      std::uint32_t input  = 0x400;
      std::uint32_t output = 0x480;
    } can_id;

    float gear_ratio = 89.0f;
    float encoder_ratio = 2048.0f;
    float threshold_stall_check = 0.25f;
    std::uint32_t control_frequency = 16000;
    bool encoder_inverted = false;
    std::uint32_t timeout_ms = 1000;
  
    float weight_low_pass_set_point = 0.2f;
    float weight_low_pass_encoder   = 0.3f;    
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
  void enable();
  void disable();

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void processRxData(const message::RxMessageDataBuffer& data);

  const Parameter _parameter;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
