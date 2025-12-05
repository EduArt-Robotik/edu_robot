/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/rpm.hpp>

#include <edu_robot/hardware/communicator.hpp>
#include <edu_robot/hardware/communicator_node.hpp>

#include <edu_robot/algorithm/low_pass_filter.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <cstdint>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

class MotorControllerHardware : public MotorController::HardwareInterface
{
public:
  struct Parameter {
    bool set_parameter = false;  //> sets and flashes parameter to motor controller hardware (EEPROM)
    std::uint32_t can_id = 0x18; //> can id used by this controller
    algorithm::LowPassFiler<float>::Parameter low_pass_set_point = {0.5f};
    float gear_ratio = 2.127659574;

    std::uint16_t max_missed_communications = 1000;
    std::uint16_t max_lag = 1200;
    std::uint8_t max_current = 0x80; //> max current unit unkown
    bool rollover_flag = false;
    std::uint8_t tic_scaling_factor = 1; //> Tic scaling factor scales the encoder tics before CAN communication 
    std::chrono::milliseconds timeout = 1000ms;
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer,
    std::shared_ptr<Communicator> communicator);
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const std::vector<Motor::Parameter>& parameter) override;
  void enable();
  void disable();
  void reset();

  static Parameter get_parameter(
    const std::string& motor_controller_name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void processRxData(const message::RxMessageDataBuffer& data);
  diagnostic::Diagnostic diagnostic() override;
  std::uint8_t getTimeStamp();
  void processSending();

  const Parameter _parameter;
  std::shared_ptr<CommunicatorNode> _communication_node;
  
  struct {
    std::vector<Rpm> rpm;    
    std::vector<Rpm> measured_rpm;
    std::chrono::system_clock::time_point stamp_last_rpm_set;
    bool timeout = true;       
    std::mutex mutex;

    algorithm::LowPassFiler<float> low_pass_set_point;
    std::chrono::time_point<std::chrono::system_clock> stamp_last_received;
    std::int32_t last_position;
    std::uint8_t error_code;
  } _processing_data;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
