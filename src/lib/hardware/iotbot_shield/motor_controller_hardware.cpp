#include "edu_robot/hardware/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/hardware/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/hardware/iot_shield/iot_shield_device_interfaces.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"

#include <edu_robot/hardware_error.hpp>
#include <edu_robot/motor_controller.hpp>

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

MotorControllerHardware::Parameter MotorControllerHardware::get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  MotorControllerHardware::Parameter parameter;

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

  parameter.gear_ratio = ros_node.get_parameter(name + ".gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(name + ".encoder_ratio").as_double();
  parameter.control_frequency = ros_node.get_parameter(name + ".control_frequency").as_int();
  parameter.timeout_ms = ros_node.get_parameter(name + ".timeout_ms").as_int();

  parameter.weight_low_pass_set_point = ros_node.get_parameter(name + ".weight_low_pass_set_point").as_double();
  parameter.weight_low_pass_encoder = ros_node.get_parameter(name + ".weight_low_pass_encoder").as_double();
  parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

  return parameter;
}

MotorControllerHardware::MotorControllerHardware(
  const std::string& name, const Parameter& parameter, std::shared_ptr<IotShieldCommunicator> communicator)
  : MotorController::HardwareInterface(name, 4)
  , IotShieldTxRxDevice(communicator)
  , _parameter(parameter)
  , _measured_rpm(4, 0.0)
{

}            

MotorControllerHardware::~MotorControllerHardware()
{

}

void MotorControllerHardware::processRxData(const uart::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  _measured_rpm[0] = -uart::message::ShieldResponse::rpm0(data);
  _measured_rpm[1] = -uart::message::ShieldResponse::rpm1(data);
  _measured_rpm[2] = -uart::message::ShieldResponse::rpm2(data);
  _measured_rpm[3] = -uart::message::ShieldResponse::rpm3(data);
  _callback_process_measurement(_measured_rpm, true);
}

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 4) {
    throw std::runtime_error("Given rpm vector needs a minimum size of 4.");
  }

  auto request = ShieldRequest::make_request<uart::message::SetRpm>(
    -rpm[0],
    -rpm[1],
    -rpm[2],
    -rpm[3]
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  future_response.get();
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  using uart::message::UART;

  auto request = ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KP>>(
    parameter.kp, 0);
  auto future_response = _communicator->sendRequest(std::move(request));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KI>>(parameter.ki, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KD>>(parameter.kd, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::SET_POINT_LOW_PASS>>(
      _parameter.weight_low_pass_set_point, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::ENCODER_LOW_PASS>>(
    _parameter.weight_low_pass_encoder, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::GEAR_RATIO>>(_parameter.gear_ratio, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::TICKS_PER_REV>>(
      _parameter.encoder_ratio, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::CONTROL_FREQUENCY>>(
      _parameter.control_frequency, 0));
  future_response.wait_for(100ms);
  future_response.get();

  // set UART timeout
  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::UART_TIMEOUT>>(
      static_cast<float>(_parameter.timeout_ms) * 1000.0f, 0
  ));
  wait_for_future(future_response, 100ms);
  future_response.get();  
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
