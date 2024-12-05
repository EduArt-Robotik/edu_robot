#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"
#include "edu_robot/diagnostic/diagnostic_level.hpp"
#include "edu_robot/hardware/communicator.hpp"
#include "edu_robot/hardware/igus/can/can_request.hpp"
#include "edu_robot/hardware/igus/can/message_definition.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"
#include "edu_robot/motor_controller.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <chrono>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

using namespace std::chrono_literals;

using hardware::igus::can::Request;
using can::message::PROTOCOL;
using can::message::SetVelocity;
using can::message::AcknowledgedVelocity;
using can::message::SetEnableMotor;
using can::message::SetDisableMotor;
using can::message::SetReset;

using can::message::GetGeneralParameter;
using can::message::GetVelocityControlParameter;
using can::message::GetFlags;
using can::message::GetWorkingHours;

// using can::message::GeneralParameter;
// using can::message::VelocityControlLoopParameter;
// using can::message::Flags;
using can::message::WorkingHours;

static void log_error_code(const std::uint8_t error_code)
{
  if (error_code & (1 << PROTOCOL::ERROR::BROWN_OUT_OR_WATCH_DOG_BIT)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "brown out or watch dog");
  }
  if (error_code & (1 << PROTOCOL::ERROR::VELOCITY_LAG_BIT)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "velocity lag");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::MOTOR_NOT_ENABLED)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "motor not enabled");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::COMM_WATCH_DOG)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "communication watch dog");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::POSITION_LAG)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "position lag");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::ENCODER_ERROR)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "encoder error");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::OVER_CURRENT)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "over current");    
  }
  if (error_code & (1 << PROTOCOL::ERROR::CAN_ERROR)) {
    RCLCPP_ERROR(rclcpp::get_logger("Igus Motor Controller"), "can error");    
  }
}

static std::string get_error_string(const std::uint8_t error_code)
{
  std::string error_string;

  if (error_code & (1 << PROTOCOL::ERROR::BROWN_OUT_OR_WATCH_DOG_BIT)) {
    error_string += "|brown out or watch dog";
  }
  if (error_code & (1 << PROTOCOL::ERROR::VELOCITY_LAG_BIT)) {
    error_string += "|velocity lag";
  }
  if (error_code & (1 << PROTOCOL::ERROR::MOTOR_NOT_ENABLED)) {
    error_string += "|motor not enabled";
  }
  if (error_code & (1 << PROTOCOL::ERROR::COMM_WATCH_DOG)) {
    error_string += "|communication watch dog";
  }
  if (error_code & (1 << PROTOCOL::ERROR::POSITION_LAG)) {
    error_string += "|position lag";    
  }
  if (error_code & (1 << PROTOCOL::ERROR::ENCODER_ERROR)) {
    error_string += "|encoder error";
  }
  if (error_code & (1 << PROTOCOL::ERROR::OVER_CURRENT)) {
    error_string += "|over current";
  }
  if (error_code & (1 << PROTOCOL::ERROR::CAN_ERROR)) {
    error_string += "|can error";    
  }

  // remove first pipe character
  return std::string(error_string.begin() + 1, error_string.end());
}

// static MotorControllerHardware::Parameter read_hardware_parameter(
//   const std::uint16_t can_id, std::shared_ptr<Communicator> communicator)
// {
//   MotorControllerHardware::Parameter parameter;

//   // General Parameter
//   {
//     auto request = Request::make_request<GetGeneralParameter>(can_id);
//     auto future_response = communicator->sendRequest(std::move(request));
//     wait_for_future(future_response, 100ms);
//     const auto got = future_response.get();

//     parameter.can_id = can_id;
//     parameter.max_current = GeneralParameter::maxCurrent(got.response());
//     parameter.max_lag = GeneralParameter::maxLag(got.response());
//     parameter.max_missed_communications = GeneralParameter::maxMissedCommunication(got.response());
//   }

//   // Flags
//   {
//     auto request = Request::make_request<GetFlags>(can_id);
//     auto future_response = communicator->sendRequest(std::move(request));
//     wait_for_future(future_response, 100ms);
//     const auto got = future_response.get();

//     parameter.rollover_flag = Flags::roolOver(got.response());
//     parameter.tic_scaling_factor = Flags::ticScale(got.response());
//   }

//   return parameter;
// }

// static Motor::Parameter read_motor_parameter(const std::uint16_t can_id, std::shared_ptr<Communicator> communicator)
// {
//   Motor::Parameter parameter;

//   // PID
//   {
//     auto request = Request::make_request<GetVelocityControlParameter>(can_id);
//     auto future_response = communicator->sendRequest(std::move(request));
//     wait_for_future(future_response, 100ms);
//     const auto got = future_response.get();

//     parameter.kp = VelocityControlLoopParameter::velocityControllerKp(got.response());
//     parameter.ki = VelocityControlLoopParameter::velocityControllerKi(got.response());
//     parameter.kd = VelocityControlLoopParameter::velocityControllerKd(got.response());
//   }

//   return parameter;
// }

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
  ros_node.declare_parameter<float>(motor_controller_name + ".gear_ratio", default_parameter.gear_ratio);

  parameter.set_parameter = ros_node.get_parameter(motor_controller_name + ".set_parameter").as_bool();
  parameter.can_id = ros_node.get_parameter(motor_controller_name + ".can_id").as_int();
  parameter.low_pass_set_point.filter_weight = ros_node.get_parameter(
    motor_controller_name + ".low_pass_set_point.filter_weight").as_double();
  parameter.gear_ratio = ros_node.get_parameter(motor_controller_name + ".gear_ratio").as_double();

  return parameter;
}

MotorControllerHardware::MotorControllerHardware(
  const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer,
  std::shared_ptr<Communicator> communicator)
  : MotorController::HardwareInterface(name, 1)
  , _parameter(parameter)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
  , _processing_data{
      { 1, 0.0 }, 
      {1, 0.0 },
      std::chrono::system_clock::now(),
      true,
      { },
      algorithm::LowPassFiler<float>(parameter.low_pass_set_point),
      std::chrono::system_clock::now(),
      0,
      0
    }
{

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
  // Initialising Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Getting Firmware Version and Working Hours
  {
    auto request = Request::make_request<GetWorkingHours>(_parameter.can_id);
    const auto got = _communication_node->sendRequest(std::move(request), 100ms);

    RCLCPP_INFO(
      rclcpp::get_logger("MotorControllerHardware(igus)"),
      "Found motor controller hardware with firmware version %u.%u",
      WorkingHours::firmwareVersion1(got.response()),
      WorkingHours::firmwareVersion2(got.response())
    );
    RCLCPP_INFO(
      rclcpp::get_logger("MotorControllerHardware(igus)"),
      "Device's working hours are: %uh %um %us",
      WorkingHours::hours(got.response()),
      WorkingHours::minutes(got.response()),
      WorkingHours::seconds(got.response())
    );
  }
  // Getting Current Position
  {
    auto request = Request::make_request<SetVelocity>(_parameter.can_id, 0, getTimeStamp());
    const auto got = _communication_node->sendRequest(std::move(request), 100ms);

    _processing_data.last_position = AcknowledgedVelocity::position(got.response());
    _processing_data.stamp_last_received = std::chrono::system_clock::now();
  }

  // Start Processing
  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware::processSending, this), 50ms
  );

  // Parameter Handling
  if (_parameter.set_parameter == true) {
    // Motor Controller Parameter
    // First check if parameter differ from stored parameter on motor controller hardware.
  }
}

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 1) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  std::scoped_lock lock(_processing_data.mutex);
  _processing_data.rpm = rpm;
  _processing_data.stamp_last_rpm_set = std::chrono::system_clock::now();
  _processing_data.timeout = false;  
}

// is called by the executer thread
void MotorControllerHardware::processSending()
{
  // Processing Setting new Set Point
  // RPM hast to be negated, because motor is turing left with positive rpm value.
  // Controller wants rotation per sections with one decimal number.
  const auto stamp_now = std::chrono::system_clock::now();
  _processing_data.mutex.lock();

  // check if time occurred
  if (_processing_data.timeout == false && stamp_now - _processing_data.stamp_last_rpm_set > _parameter.timeout) {
    // timeout occurred --> reset rpm values to zero
    std::fill(_processing_data.rpm.begin(), _processing_data.rpm.end(), 0.0f);
    _processing_data.timeout = true;
    disable();
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "timeout occurred! --> disable motor controller.");
  }

  // low pass filter input set point
  _processing_data.low_pass_set_point(-_processing_data.rpm[0].rps() * 10.0f * _parameter.gear_ratio);

  // sending velocity
  auto request = Request::make_request<SetVelocity>(
    _parameter.can_id,
    static_cast<std::uint8_t>(_processing_data.low_pass_set_point.getValue() + 127.5f), // \todo move converting to message definition. CanRequest ist the problem!
    // Rpm(_processing_data.low_pass_set_point.getValue()),
    getTimeStamp()
  );
  _processing_data.mutex.unlock();

  // Communicate
  const auto got = _communication_node->sendRequest(std::move(request), 100ms);

  // Processing Received Feedback
  const auto stamp_received = std::chrono::system_clock::now();
  const auto stamp_diff = std::chrono::duration_cast<std::chrono::milliseconds>(stamp_received - _processing_data.stamp_last_received).count();
  const float dt = std::min(static_cast<float>(stamp_diff) / 1000.0f, 0.5f); // limit dt to 0.5s
  const auto current_position = AcknowledgedVelocity::position(got.response());
  const float position_diff = static_cast<float>(current_position - _processing_data.last_position);
  const float position_per_seconds = position_diff / dt;
  const float rotation_per_seconds = position_per_seconds / 13188.7f; // number was measured on real platform

  // Measured rpm value has to be negated, because motor is turing left with positive rpm value.
  std::scoped_lock lock(_processing_data.mutex);
  _processing_data.measured_rpm[0] = Rpm::fromRps(rotation_per_seconds / _parameter.gear_ratio) * -1.0f;
  _processing_data.last_position = current_position;
  _processing_data.stamp_last_received = stamp_received;

  // Sending Feedback to Layer above
  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(_processing_data.measured_rpm, AcknowledgedVelocity::enabled(got.response()));

  // Handling Error
  _processing_data.error_code = AcknowledgedVelocity::errorCode(got.response());

  if (_processing_data.error_code & ~(1 << PROTOCOL::ERROR::MOTOR_NOT_ENABLED)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Igus Motor Controller"),
      "Received error code %u from hardware with can id = %u", _processing_data.error_code, _parameter.can_id
    );
    log_error_code(_processing_data.error_code);
  }
}

void MotorControllerHardware::enable()
{
  auto request = Request::make_request<SetEnableMotor>(_parameter.can_id, 0);
  _communication_node->sendRequest(std::move(request), 200ms);

  std::scoped_lock lock(_processing_data.mutex);
  _processing_data.low_pass_set_point.clear();  
}

void MotorControllerHardware::disable()
{
  auto request = Request::make_request<SetDisableMotor>(_parameter.can_id, 0);
  _communication_node->sendRequest(std::move(request), 200ms);
}

void MotorControllerHardware::reset()
{
  auto request = Request::make_request<SetReset>(_parameter.can_id, 0);
  _communication_node->sendRequest(std::move(request), 200ms);

  std::scoped_lock lock(_processing_data.mutex);
  _processing_data.error_code = 0;
  _processing_data.low_pass_set_point.clear();
}

diagnostic::Diagnostic MotorControllerHardware::diagnostic()
{
  diagnostic::Diagnostic diagnostic;
  std::scoped_lock lock(_processing_data.mutex);

  if (_processing_data.error_code != 0) {
    diagnostic.add(
      "hardware", get_error_string(_processing_data.error_code), diagnostic::Level::ERROR
    );  
  }
  else {
    diagnostic.add("hardware", "working properly", diagnostic::Level::OK);
  }

  return diagnostic;
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
