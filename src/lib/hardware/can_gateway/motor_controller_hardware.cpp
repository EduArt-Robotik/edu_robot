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
  {
    auto request = Request::make_request<SetTimeout>(
      hardware_parameter.can_id.input, hardware_parameter.timeout.count());
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetInvertedEncoder>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_inverted);
    communication_node->sendRequest(std::move(request), 100ms);
  }  
  {
    if (parameter[0].closed_loop) {
      auto request = Request::make_request<v1::SetClosedLoop>(hardware_parameter.can_id.input);
      communication_node->sendRequest(std::move(request), 100ms);
    }
    else {
      auto request = Request::make_request<v1::SetOpenLoop>(hardware_parameter.can_id.input);
      communication_node->sendRequest(std::move(request), 100ms);    
    }
  }
  {
    auto request = Request::make_request<SetFrequency>(
      hardware_parameter.can_id.input, hardware_parameter.control_frequency);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetCtlKp>(
      hardware_parameter.can_id.input, parameter[0].kp);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetCtlKi>(
      hardware_parameter.can_id.input, parameter[0].ki);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetCtlKd>(
      hardware_parameter.can_id.input, parameter[0].kd);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetCtlAntiWindUp>(
      hardware_parameter.can_id.input, true);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetCtlInputFilter>(
      hardware_parameter.can_id.input, hardware_parameter.input_filter_weight);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetGearRatio>(
      hardware_parameter.can_id.input, hardware_parameter.gear_ratio);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetTicksPerRevision>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_ratio);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<v1::SetRpmMax>(
      hardware_parameter.can_id.input, parameter[0].max_rpm);
    communication_node->sendRequest(std::move(request), 100ms);
  }
}

void initialize_controller_firmware_v0_3(
  const std::vector<Motor::Parameter>& parameter, const MotorControllerHardware::Parameter& hardware_parameter,
  std::shared_ptr<CommunicatorNode> communication_node)
{
  // timeout
  {
    auto request = Request::make_request<SetTimeout>(
      hardware_parameter.can_id.input, hardware_parameter.timeout.count());
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // gear ratio
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetGearRatio>(
      hardware_parameter.can_id.input, hardware_parameter.gear_ratio, channel);
    communication_node->sendRequest(std::move(request), 100ms);    
  }
  // encoder ratio
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetTicksPerRevision>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_ratio, channel);
    communication_node->sendRequest(std::move(request), 100ms);    
  }
  // inverted encoder motor
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetInvertedEncoder>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_inverted, channel);
    communication_node->sendRequest(std::move(request), 100ms);    
  }
  // input filter weight
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetCtlInputFilter>(
      hardware_parameter.can_id.input, hardware_parameter.input_filter_weight, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // closed loop
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetClosedLoop>(
      hardware_parameter.can_id.input, parameter[channel].closed_loop, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // max rpm
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetRpmMax>(
      hardware_parameter.can_id.input, parameter[channel].max_rpm, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // kp
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetCtlKp>(
      hardware_parameter.can_id.input, parameter[channel].kp, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // ki
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetCtlKi>(
      hardware_parameter.can_id.input, parameter[channel].ki, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  // kd
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request<v2::SetCtlKd>(
      hardware_parameter.can_id.input, parameter[channel].kd, channel);
    communication_node->sendRequest(std::move(request), 100ms);
  }

  // checking set parameter by reading them back
  {
    auto request = Request::make_request_with_response<v2::GetTimeout>(
      hardware_parameter.can_id.input, hardware_parameter.can_id.output);
    const auto got = communication_node->sendRequest(std::move(request), 200ms);
    const auto timeout_ms = v2::Timeout::timeoutMs(got.response());
    if (timeout_ms != hardware_parameter.timeout.count()) {
      throw HardwareError(State::MOTOR_ERROR, "Failed to set timeout parameter.");
    }
  }
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request_with_response<v2::GetCtlKp>(
      hardware_parameter.can_id.input, hardware_parameter.can_id.output, channel);
    const auto got = communication_node->sendRequest(std::move(request), 200ms);
    const auto kp = v2::CtlKp::kp(got.response());
    if (std::abs(kp - parameter[channel].kp) > 1e-5f) {
      throw HardwareError(State::MOTOR_ERROR, "Failed to set controller kp parameter.");
    }
  }
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request_with_response<v2::GetCtlKi>(
      hardware_parameter.can_id.input, hardware_parameter.can_id.output, channel);
    const auto got = communication_node->sendRequest(std::move(request), 200ms);
    const auto ki = v2::CtlKi::ki(got.response());
    if (std::abs(ki - parameter[channel].ki) > 1e-5f) {
      throw HardwareError(State::MOTOR_ERROR, "Failed to set controller ki parameter.");
    }
  }
  for (std::size_t channel = 0; channel < parameter.size(); ++channel) {
    auto request = Request::make_request_with_response<v2::GetCtlKd>(
      hardware_parameter.can_id.input, hardware_parameter.can_id.output, channel);
    const auto got = communication_node->sendRequest(std::move(request), 200ms);
    const auto kd = v2::CtlKd::kd(got.response());
    if (std::abs(kd - parameter[channel].kd) > 1e-5f) {
      throw HardwareError(State::MOTOR_ERROR, "Failed to set controller kd parameter.");
    }
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
