/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>
#include <edu_robot/rpm.hpp>

#include <edu_robot/hardware/communicator.hpp>
#include <edu_robot/hardware/communicator_device_interfaces.hpp>

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
                              , public CommunicatorTxRxDevice
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
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, 1)
    , CommunicatorTxRxDevice(communicator)
    , _parameter(parameter)
    , _processing_data{
        std::vector<Rpm>(1, 0.0),
        algorithm::LowPassFiler<float>(parameter.low_pass_set_point),
        std::chrono::system_clock::now(),
        0,
        0
      }
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void enable();
  void disable();
  void reset();

  static Parameter get_parameter(
    const std::string& motor_controller_name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void processRxData(const message::RxMessageDataBuffer& data);
  diagnostic::Diagnostic diagnostic() override;
  std::uint8_t getTimeStamp();

  const Parameter _parameter;
  
  struct {
    std::vector<Rpm> measured_rpm;
    algorithm::LowPassFiler<float> low_pass_set_point;
    std::chrono::time_point<std::chrono::system_clock> stamp_last_received;
    std::uint32_t last_position;
    std::uint8_t error_code;
  } _processing_data;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
