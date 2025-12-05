/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/executer.hpp>
#include <edu_robot/rpm.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class MotorControllerHardware : public MotorController::HardwareInterface
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
    std::chrono::milliseconds timeout = 1000ms;
  
    float input_filter_weight = 0.2f;
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer,
    std::shared_ptr<Communicator> communicator);
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const std::vector<Motor::Parameter>& parameter) override;
  void enable();
  void disable();

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

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
