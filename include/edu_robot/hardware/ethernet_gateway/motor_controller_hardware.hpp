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
namespace ethernet {

template <std::size_t NUM_CHANNELS>
class MotorControllerHardware : public MotorController::HardwareInterface
                              , public CommunicatorTxRxDevice
{
public:
  struct Parameter {
    std::uint8_t can_id = 0;
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
    : MotorController::HardwareInterface(name, NUM_CHANNELS)
    , CommunicatorTxRxDevice(communicator)
    , _parameter(parameter)
    , _measured_rpm(NUM_CHANNELS, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
  {
    MotorControllerHardware::Parameter parameter;

    ros_node.declare_parameter<int>(name + ".can_id", default_parameter.can_id);

    ros_node.declare_parameter<float>(
      name + ".gear_ratio", default_parameter.gear_ratio);
    ros_node.declare_parameter<float>(
      name + ".encoder_ratio", default_parameter.encoder_ratio);
    ros_node.declare_parameter<int>(
      name + ".control_frequency", default_parameter.control_frequency);
    ros_node.declare_parameter<int>(
      name + ".timeout_ms", default_parameter.timeout_ms);  

    ros_node.declare_parameter<float>(
      name + ".weight_low_pass_set_point", default_parameter.weight_low_pass_set_point);
    ros_node.declare_parameter<float>(
      name + ".weight_low_pass_encoder", default_parameter.weight_low_pass_encoder);
    ros_node.declare_parameter<bool>(
      name + ".encoder_inverted", default_parameter.encoder_inverted);

    parameter.can_id = ros_node.get_parameter(name + ".can_id").as_int();

    parameter.gear_ratio = ros_node.get_parameter(name + ".gear_ratio").as_double();
    parameter.encoder_ratio = ros_node.get_parameter(name + ".encoder_ratio").as_double();
    parameter.control_frequency = ros_node.get_parameter(name + ".control_frequency").as_int();
    parameter.timeout_ms = ros_node.get_parameter(name + ".timeout_ms").as_int();

    parameter.weight_low_pass_set_point = ros_node.get_parameter(name + ".weight_low_pass_set_point").as_double();
    parameter.weight_low_pass_encoder = ros_node.get_parameter(name + ".weight_low_pass_encoder").as_double();
    parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

    return parameter;    
  }

private:
  void processRxData(const message::RxMessageDataBuffer& data);

  const Parameter _parameter;
  std::uint8_t _can_id;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
