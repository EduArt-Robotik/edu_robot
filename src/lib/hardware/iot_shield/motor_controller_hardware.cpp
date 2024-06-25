#include "edu_robot/hardware/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/hardware/iot_shield/uart/message_definition.hpp"
#include "edu_robot/hardware/iot_shield/uart/uart_request.hpp"
#include "edu_robot/hardware/rx_data_endpoint.hpp"

#include <chrono>
#include <edu_robot/hardware/communicator_node.hpp>

#include <edu_robot/hardware_error.hpp>
#include <edu_robot/motor_controller.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using namespace std::chrono_literals;

using uart::Request;
using uart::message::ShieldResponse;

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
    name + ".timeout_ms", default_parameter.timeout.count());  

  ros_node.declare_parameter<float>(
    name + ".weight_low_pass_set_point", default_parameter.weight_low_pass_set_point);
  ros_node.declare_parameter<float>(
    name + ".weight_low_pass_encoder", default_parameter.weight_low_pass_encoder);
  ros_node.declare_parameter<bool>(
    name + ".encoder_inverted", default_parameter.encoder_inverted);

  parameter.gear_ratio = ros_node.get_parameter(name + ".gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(name + ".encoder_ratio").as_double();
  parameter.control_frequency = ros_node.get_parameter(name + ".control_frequency").as_int();
  parameter.timeout = std::chrono::milliseconds(ros_node.get_parameter(name + ".timeout_ms").as_int());

  parameter.weight_low_pass_set_point = ros_node.get_parameter(name + ".weight_low_pass_set_point").as_double();
  parameter.weight_low_pass_encoder = ros_node.get_parameter(name + ".weight_low_pass_encoder").as_double();
  parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

  return parameter;
}

MotorControllerHardware::MotorControllerHardware(
  const std::string& name, const Parameter& parameter, std::shared_ptr<Executer> executer,
  std::shared_ptr<Communicator> communicator)
  : MotorController::HardwareInterface(name, 4)
  , _parameter(parameter)
  , _communication_node(std::make_shared<CommunicatorNode>(executer, communicator))
  , _data{
      {4, 0.0},
      {4, 0.0},
      std::chrono::system_clock::now(),
      true,
      {}
    }
{
  _communication_node->createRxDataEndPoint<RxDataEndPoint, ShieldResponse>(
    std::bind(&MotorControllerHardware::processRxData, this, std::placeholders::_1), 3
  );
  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware::processSending, this), 100ms
  );
}            

MotorControllerHardware::~MotorControllerHardware()
{

}

// is called by the rx data endpoint thread
void MotorControllerHardware::processRxData(const uart::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  // std::lock_guard lock(_data.mutex);
  // no need for mutex lock, because measured rpm are only touched here
  std::scoped_lock lock(_data.mutex);
  _data.measured_rpm[0] = -uart::message::ShieldResponse::rpm0(data);
  _data.measured_rpm[1] = -uart::message::ShieldResponse::rpm1(data);
  _data.measured_rpm[2] = -uart::message::ShieldResponse::rpm2(data);
  _data.measured_rpm[3] = -uart::message::ShieldResponse::rpm3(data);
  _callback_process_measurement(_data.measured_rpm, !_data.timeout);
}

// is called by the main thread
void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 4) {
    throw std::runtime_error("Given rpm vector needs a minimum size of 4.");
  }

  std::lock_guard lock(_data.mutex);
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
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "timeout occurred! --> disable motor controller.");
  }

  auto request = Request::make_request<uart::message::SetRpm>(
    -_data.rpm[0],
    -_data.rpm[1],
    -_data.rpm[2],
    -_data.rpm[3]
  );
  _data.mutex.unlock();

  _communication_node->sendRequest(std::move(request), 100ms);
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  using uart::message::UART;

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::KP>>(
      parameter.kp, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::KI>>(parameter.ki, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::KD>>(parameter.kd, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::SET_POINT_LOW_PASS>>(
      _parameter.weight_low_pass_set_point, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::ENCODER_LOW_PASS>>(
    _parameter.weight_low_pass_encoder, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::GEAR_RATIO>>(
      _parameter.gear_ratio, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::TICKS_PER_REV>>(
      _parameter.encoder_ratio, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::CONTROL_FREQUENCY>>(
      _parameter.control_frequency, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }

  {
    // set UART timeout
    auto request = Request::make_request<uart::message::SetValueF<UART::COMMAND::SET::UART_TIMEOUT>>(
      static_cast<float>(_parameter.timeout.count()) * 1000.0f, 0);
    _communication_node->sendRequest(std::move(request), 100ms);
  }
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
