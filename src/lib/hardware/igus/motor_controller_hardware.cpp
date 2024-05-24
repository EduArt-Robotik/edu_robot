#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"
#include "edu_robot/hardware/igus/can/can_request.hpp"
#include "edu_robot/hardware/igus/can/message_definition.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

using namespace std::chrono_literals;

using hardware::igus::can::CanRequest;
using can::message::PROTOCOL;
using can::message::SetVelocity;
using can::message::AcknowledgedVelocity;
using can::message::SetEnableMotor;
using can::message::SetDisableMotor;
using can::message::SetReset;

MotorControllerHardware::Parameter MotorControllerHardware::get_parameter(
    const std::string& motor_controller_name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<bool>(
    motor_controller_name + ".set_parameter", default_parameter.set_parameter);
  ros_node.declare_parameter<int>(motor_controller_name + ".can_id", default_parameter.can_id);
  ros_node.declare_parameter<float>(
    motor_controller_name + ".low_pass_set_point.filter_weight",
    default_parameter.low_pass_set_point.filter_weight);

  parameter.set_parameter = ros_node.get_parameter(motor_controller_name + ".set_parameter").as_bool();
  parameter.can_id = ros_node.get_parameter(motor_controller_name + ".can_id").as_int();
  parameter.low_pass_set_point.filter_weight = ros_node.get_parameter(
    motor_controller_name + ".low_pass_set_point.filter_weight").as_double();

  return parameter;
}

void MotorControllerHardware::processRxData(const can::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr
      ||
      can::message::AcknowledgedVelocity::canId(data) != _parameter.can_id)
  {
    return;
  }
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }
  if (_parameter.set_parameter == false) {
    // do not flash set and flash the parameter to the EEPROM
    return;
  }

  // Motor Controller Parameter
}

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 1) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  _low_pass_set_point(rpm[0]);
  auto request = CanRequest::make_request<SetVelocity>(
    _parameter.can_id,
    static_cast<std::uint8_t>(_low_pass_set_point.getValue() + 127.0f), // \todo move converting to message definition. CanRequest ist the problem!
    getTimeStamp()
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
  _measured_rpm[0] = AcknowledgedVelocity::position(got.response());

  if (_callback_process_measurement == nullptr) {
    return;
  }
  if (AcknowledgedVelocity::errorCode(got.response()) & ~PROTOCOL::ERROR::MOTOR_NOT_ENABLED) {
    // \todo do some error handling here
  }

  _callback_process_measurement(_measured_rpm, AcknowledgedVelocity::enabled(got.response()));
}

void MotorControllerHardware::enable()
{
  auto request = CanRequest::make_request<SetEnableMotor>(_parameter.can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}
  
void MotorControllerHardware::disable()
{
  auto request = CanRequest::make_request<SetDisableMotor>(_parameter.can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}

void MotorControllerHardware::reset()
{
  auto request = CanRequest::make_request<SetReset>(_parameter.can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}

std::uint8_t MotorControllerHardware::getTimeStamp()
{
  const auto stamp = std::chrono::system_clock::now();
  const auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(stamp).time_since_epoch().count() % 256;

  return ms;
}

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
