#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/rx_data_endpoint.hpp>

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>

#include <edu_robot/hardware_error.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using can::Request;
using can::CanRxDataEndPoint;

using namespace can::message::motor_controller;

/**
 * Parameter Handler for Motor Controller Parameter Setting and Validation
 */
struct parameter_handler {
  parameter_handler(
    const MotorControllerHardware::Parameter& hardware_parameter,
    const std::shared_ptr<CommunicatorNode> communication_node)
  : _hardware_parameter(hardware_parameter)
  , _communication_node(communication_node)
{ }

template <typename SetMessage, typename Value>
void set_parameter(const Value& value) {
  auto request = Request::make_request<SetMessage>(
    _hardware_parameter.can_id.input, value);
  _communication_node->sendRequest(std::move(request), 100ms);    
}

template <typename SetMessage, typename Value>
void set_channel_parameter(const Value& value, const std::size_t channel) {
  auto request = Request::make_request<SetMessage>(
    _hardware_parameter.can_id.input, value, channel);
  _communication_node->sendRequest(std::move(request), 100ms);
}

template <typename GetMessage, typename ResponseMessage, typename Value>
void valid_parameter(const Value& expected_value) {
  auto request = Request::make_request_with_response<GetMessage>(
    _hardware_parameter.can_id.input, _hardware_parameter.can_id.output);
  const auto got = _communication_node->sendRequest(std::move(request), 200ms);
  const auto value = ResponseMessage::value(got.response());

  if (value != expected_value) {
    throw HardwareError(
      State::MOTOR_ERROR, std::string("Failed to set timeout parameter \"") + ResponseMessage::name() + "\"."
    );
  }  
}

template <typename GetMessage, typename ResponseMessage, typename Value>
void valid_channel_parameter(const Value& expected_value, const std::size_t channel) {
  auto request = Request::make_request_with_response<GetMessage>(
    _hardware_parameter.can_id.input, _hardware_parameter.can_id.output, channel);
  const auto got = _communication_node->sendRequest(std::move(request), 200ms);
  const auto value = ResponseMessage::value(got.response());

  if (value != expected_value) {
    throw HardwareError(
      State::MOTOR_ERROR, std::string("Failed to set channel parameter \"") + ResponseMessage::name() + "\"."
    );
  }
}

private:
  const MotorControllerHardware::Parameter& _hardware_parameter;
  const std::shared_ptr<CommunicatorNode> _communication_node;
};

MotorControllerHardware::Parameter MotorControllerHardware::get_parameter(
  const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  MotorControllerHardware::Parameter parameter;

  ros_node.declare_parameter<int>(
    name + ".can_id.input", default_parameter.can_id.input);
  ros_node.declare_parameter<int>(
    name + ".can_id.output", default_parameter.can_id.output);

  ros_node.declare_parameter<float>(
    name + ".gear_ratio", default_parameter.gear_ratio);
  ros_node.declare_parameter<float>(
    name + ".encoder_ratio", default_parameter.encoder_ratio);
  ros_node.declare_parameter<int>(
    name + ".control_frequency", default_parameter.control_frequency);
  ros_node.declare_parameter<int>(
    name + ".timeout_ms", default_parameter.timeout.count());  

  ros_node.declare_parameter<float>(
    name + ".input_filter_weight", default_parameter.input_filter_weight);
  ros_node.declare_parameter<bool>(
    name + ".encoder_inverted", default_parameter.encoder_inverted);

  parameter.can_id.input = ros_node.get_parameter(name + ".can_id.input").as_int();
  parameter.can_id.output = ros_node.get_parameter(name + ".can_id.output").as_int();

  parameter.gear_ratio = ros_node.get_parameter(name + ".gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(name + ".encoder_ratio").as_double();
  parameter.control_frequency = ros_node.get_parameter(name + ".control_frequency").as_int();
  parameter.timeout = std::chrono::milliseconds(ros_node.get_parameter(name + ".timeout_ms").as_int());

  parameter.input_filter_weight = ros_node.get_parameter(name + ".input_filter_weight").as_double();
  parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

  return parameter;
}

void initialize_controller_firmware_v0_2(
  const std::vector<Motor::Parameter>& parameter, const MotorControllerHardware::Parameter& hardware_parameter,
  std::shared_ptr<CommunicatorNode> communication_node)
{
  parameter_handler handler(hardware_parameter, communication_node);

  handler.set_parameter<SetTimeout>(hardware_parameter.timeout.count());
  handler.set_parameter<v1::SetInvertedEncoder>(hardware_parameter.encoder_inverted);
  handler.set_parameter<SetFrequency>(hardware_parameter.control_frequency);
  handler.set_parameter<v1::SetCtlKp>(parameter[0].pid.kp);
  handler.set_parameter<v1::SetCtlKi>(parameter[0].pid.ki);
  handler.set_parameter<v1::SetCtlKd>(parameter[0].pid.kd);
  handler.set_parameter<v1::SetCtlAntiWindUp>(true);
  handler.set_parameter<v1::SetCtlInputFilter>(hardware_parameter.input_filter_weight);
  handler.set_parameter<v1::SetGearRatio>(hardware_parameter.gear_ratio);
  handler.set_parameter<v1::SetTicksPerRevision>(hardware_parameter.encoder_ratio);
  handler.set_parameter<v1::SetRpmMax>(parameter[0].max_rpm);

  if (parameter[0].closed_loop) {
    auto request = Request::make_request<v1::SetClosedLoop>(hardware_parameter.can_id.input);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  else {
    auto request = Request::make_request<v1::SetOpenLoop>(hardware_parameter.can_id.input);
    communication_node->sendRequest(std::move(request), 100ms);    
  }
}

void initialize_controller_firmware_v0_3(
  const std::vector<Motor::Parameter>& parameter, const MotorControllerHardware::Parameter& hardware_parameter,
  std::shared_ptr<CommunicatorNode> communication_node)
{
  parameter_handler handler(hardware_parameter, communication_node);

  // setting parameters
  handler.set_parameter<SetTimeout>(hardware_parameter.timeout.count());

  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    handler.set_channel_parameter<v2::SetGearRatio>(hardware_parameter.gear_ratio, channel);
    handler.set_channel_parameter<v2::SetTicksPerRevision>(hardware_parameter.encoder_ratio, channel);
    handler.set_channel_parameter<v2::SetInvertedEncoder>(hardware_parameter.encoder_inverted, channel);
    handler.set_channel_parameter<v2::SetCtlInputFilter>(hardware_parameter.input_filter_weight, channel);
    handler.set_channel_parameter<v2::SetClosedLoop>(parameter[channel].closed_loop, channel);
    handler.set_channel_parameter<v2::SetRpmMax>(parameter[channel].max_rpm, channel);
    handler.set_channel_parameter<v2::SetCtlKp>(parameter[channel].pid.kp, channel);
    handler.set_channel_parameter<v2::SetCtlKi>(parameter[channel].pid.ki, channel);
    handler.set_channel_parameter<v2::SetCtlKd>(parameter[channel].pid.kd, channel);
  }

  // checking set parameter by reading them back
  handler.valid_parameter<v2::GetTimeout, v2::Timeout>(hardware_parameter.timeout.count());

  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    handler.valid_channel_parameter<v2::GetGearRatio, v2::GearRatio>(hardware_parameter.gear_ratio, channel);
    handler.valid_channel_parameter<v2::GetTicksPerRevision, v2::TicksPerRevision>(hardware_parameter.encoder_ratio, channel);
    handler.valid_channel_parameter<v2::GetInvertedEncoder, v2::InvertedEncoder>(hardware_parameter.encoder_inverted, channel);
    handler.valid_channel_parameter<v2::GetCtlInputFilter, v2::CtlInputFilter>(hardware_parameter.input_filter_weight, channel);
    handler.valid_channel_parameter<v2::GetClosedLoop, v2::ClosedLoop>(parameter[channel].closed_loop, channel);
    handler.valid_channel_parameter<v2::GetRpmMax, v2::RpmMax>(parameter[channel].max_rpm, channel);
    handler.valid_channel_parameter<v2::GetCtlKp, v2::CtlKp>(parameter[channel].pid.kp, channel);
    handler.valid_channel_parameter<v2::GetCtlKi, v2::CtlKi>(parameter[channel].pid.ki, channel);
    handler.valid_channel_parameter<v2::GetCtlKd, v2::CtlKd>(parameter[channel].pid.kd, channel);
  }
}




MotorControllerHardware::MotorControllerHardware(
  const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer,
  std::shared_ptr<Communicator> communicator)
  : MotorController::HardwareInterface(name, 2)
  , _parameter(parameter)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
  , _data{
      { 2, 0.0 },
      { 2, 0.0 },
      std::chrono::system_clock::now(),
      true,
      { }
    }
{

}

// is called by executer thread
void MotorControllerHardware::processRxData(const message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || Response::canId(data) != _parameter.can_id.output) {
    return;
  }
  
  // measured rpm only used here
  _data.measured_rpm[0] = Response::rpm0(data);
  _data.measured_rpm[1] = Response::rpm1(data);  
  _callback_process_measurement(_data.measured_rpm, Response::enabled(data));
}

void MotorControllerHardware::initialize(const std::vector<Motor::Parameter>& parameters)
{
  // Initial Motor Controller Hardware
  if (parameters.size() != motors()) {
    throw std::invalid_argument("given number of motors is wrong.");
  }

  try {
    auto request = Request::make_request_with_response<v2::GetFirmware>(
      _parameter.can_id.input, _parameter.can_id.output);
    const auto got = _communication_node->sendRequest(std::move(request), 200ms);

    const auto major = v2::Firmware::major(got.response());
    const auto minor = v2::Firmware::minor(got.response());
    const auto patch = v2::Firmware::patch(got.response());

    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "detected motor controller firmware v%u.%u.%u", major, minor, patch);
    initialize_controller_firmware_v0_3(parameters, _parameter, _communication_node);
  }
  catch (...) {
    // firmware version request failed --> fall back to v0.2.x initialization
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "could not get motor controller firmware version. assume v0.2.x");
    initialize_controller_firmware_v0_2(parameters, _parameter, _communication_node);
  }

  // Starting Continuous Communication with Motor Controller
  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware::processSending, this), 50ms
  );
  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, Response>(
    _parameter.can_id.output,
    std::bind(&MotorControllerHardware::processRxData, this, std::placeholders::_1)
  );
}

void MotorControllerHardware::processSetValue(const std::vector<robot::Rpm>& rpm)
{
  if (rpm.size() < 2) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  std::scoped_lock lock(_data.mutex);
  _data.rpm = rpm;
  _data.stamp_last_rpm_set = std::chrono::system_clock::now();
  _data.timeout = false;  
}

void MotorControllerHardware::processSending()
{
  const auto stamp_now = std::chrono::system_clock::now();
  _data.mutex.lock();

  if (_data.timeout == false && stamp_now - _data.stamp_last_rpm_set > _parameter.timeout) {
    // timeout occurred --> reset rpm values to zero
    std::fill(_data.rpm.begin(), _data.rpm.end(), 0.0f);
    _data.timeout = true;
    disable();
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "timeout occurred! --> disable motor controller.");
  }

  auto request = Request::make_request<SetRpm>(
    _parameter.can_id.input,
    _data.rpm[0],
    _data.rpm[1]
  );
  _data.mutex.unlock();

  _communication_node->sendRequest(std::move(request), 100ms);
}

void MotorControllerHardware::enable()
{
  auto request = Request::make_request<Enable>(_parameter.can_id.input);
  _communication_node->sendRequest(std::move(request), 100ms);
}

void MotorControllerHardware::disable()
{
  auto request = Request::make_request<Disable>(_parameter.can_id.input);
  _communication_node->sendRequest(std::move(request), 100ms);
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
