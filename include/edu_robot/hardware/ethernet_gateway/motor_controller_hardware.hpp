/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/communicator_node.hpp"

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/executer.hpp>
#include <edu_robot/rpm.hpp>

#include <memory>
#include <chrono>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

template <std::size_t NUM_CHANNELS>
class MotorControllerHardware : public MotorController::HardwareInterface
{
public:
  struct Parameter {
    std::uint8_t can_id = 0;
    float gear_ratio = 89.0f;
    float encoder_ratio = 2048.0f;
    float threshold_stall_check = 0.25f;
    std::uint32_t control_frequency = 16000;
    bool encoder_inverted = false;
    std::chrono::milliseconds timeout = 1000ms;
  
    float weight_low_pass_set_point = 0.2f;
    float weight_low_pass_encoder   = 0.3f;    
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer, 
    std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, NUM_CHANNELS)
    , _parameter(parameter)
    , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
    , _data{
        { NUM_CHANNELS, 0.0 },
        { NUM_CHANNELS, 0.0 },
        std::chrono::system_clock::now(),
        true,
        { }
      }
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override
  {
    if (rpm.size() < 1) {
      throw std::runtime_error("Given RPM vector is too small.");
    }

    std::lock_guard lock(_data.mutex);
    _data.rpm = rpm;
    _data.stamp_last_rpm_set = std::chrono::system_clock::now();
    _data.timeout = false;    
  }
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
      name + ".timeout_ms", default_parameter.timeout.count());  

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
    parameter.timeout = std::chrono::milliseconds(ros_node.get_parameter(name + ".timeout_ms").as_int());

    parameter.weight_low_pass_set_point = ros_node.get_parameter(name + ".weight_low_pass_set_point").as_double();
    parameter.weight_low_pass_encoder = ros_node.get_parameter(name + ".weight_low_pass_encoder").as_double();
    parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

    return parameter;    
  }

private:
  void processRxData(const message::RxMessageDataBuffer& data);
  void processSending();

  const Parameter _parameter;
  std::shared_ptr<CommunicatorNode> _communication_node;

  struct {
    std::vector<Rpm> rpm;
    std::vector<Rpm> measured_rpm;
    std::chrono::system_clock::time_point stamp_last_rpm_set;
    bool timeout = true;    
    std::mutex mutex;
  } _data;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
