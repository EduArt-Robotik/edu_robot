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

#include <memory>

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
  };

  MotorControllerHardware(
    const std::string& name, const Parameter& parameter, std::shared_ptr<Communicator> communicator)
    : MotorController::HardwareInterface(name, 1)
    , CommunicatorTxRxDevice(communicator)
    , _parameter(parameter)
    , _measured_rpm(1, 0.0)
    , _low_pass_set_point(parameter.low_pass_set_point)
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
  std::uint8_t getTimeStamp();

  const Parameter _parameter;
  std::vector<Rpm> _measured_rpm;
  algorithm::LowPassFiler<float> _low_pass_set_point;
};

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
