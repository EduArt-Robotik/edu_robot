/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/rpm.hpp>

#include "edu_robot/hardware/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/hardware/iot_shield/iot_shield_device_interfaces.hpp"

#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace iotbot {

class MotorControllerHardware : public MotorController::HardwareInterface
                              , public eduart::robot::iotbot::IotShieldTxRxDevice
{
public:
  struct Parameter {
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
    const std::string& name, const Parameter& parameter, std::shared_ptr<IotShieldCommunicator> communicator);
  ~MotorControllerHardware() override;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  const Parameter& _parameter;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
