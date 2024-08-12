/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/executer.hpp"

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/rpm.hpp>

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/communicator.hpp>

#include <memory>
#include <string>
#include <chrono>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class MotorControllerHardware : public MotorController::HardwareInterface
{
public:
  struct Parameter {
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
    std::shared_ptr<Communicator> communicator);
  ~MotorControllerHardware() override;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;

  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

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

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
