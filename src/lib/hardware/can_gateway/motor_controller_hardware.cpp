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

using can::message::motor_controller::Enable;
using can::message::motor_controller::Disable;
using can::message::motor_controller::SetTimeout;
using can::message::motor_controller::SetInvertedEncoder;
using can::message::motor_controller::SetPwm;
using can::message::motor_controller::SetRpm;
using can::message::motor_controller::SetOpenLoop;
using can::message::motor_controller::SetClosedLoop;
using can::message::motor_controller::SetFrequency;
using can::message::motor_controller::SetCtlKp;
using can::message::motor_controller::SetCtlKi;
using can::message::motor_controller::SetCtlKd;
using can::message::motor_controller::SetCtlAntiWindUp;
using can::message::motor_controller::SetCtlInputFilter;
using can::message::motor_controller::SetGearRatio;
using can::message::motor_controller::SetTicksPerRevision;
using can::message::motor_controller::SetRpmMax;
using can::message::motor_controller::Response;

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
    name + ".timeout_ms", default_parameter.timeout_ms);  

  ros_node.declare_parameter<float>(
    name + ".weight_low_pass_set_point", default_parameter.weight_low_pass_set_point);
  ros_node.declare_parameter<float>(
    name + ".weight_low_pass_encoder", default_parameter.weight_low_pass_encoder);
  ros_node.declare_parameter<bool>(
    name + ".encoder_inverted", default_parameter.encoder_inverted);

  parameter.can_id.input = ros_node.get_parameter(name + ".can_id.input").as_int();
  parameter.can_id.output = ros_node.get_parameter(name + ".can_id.output").as_int();

  parameter.gear_ratio = ros_node.get_parameter(name + ".gear_ratio").as_double();
  parameter.encoder_ratio = ros_node.get_parameter(name + ".encoder_ratio").as_double();
  parameter.control_frequency = ros_node.get_parameter(name + ".control_frequency").as_int();
  parameter.timeout_ms = ros_node.get_parameter(name + ".timeout_ms").as_int();

  parameter.weight_low_pass_set_point = ros_node.get_parameter(name + ".weight_low_pass_set_point").as_double();
  parameter.weight_low_pass_encoder = ros_node.get_parameter(name + ".weight_low_pass_encoder").as_double();
  parameter.encoder_inverted = ros_node.get_parameter(name + ".encoder_inverted").as_bool();

  return parameter;
}

void initialize_controller(
  const Motor::Parameter& parameter, const MotorControllerHardware::Parameter& hardware_parameter,
  std::shared_ptr<CommunicatorNode> communication_node)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  {
    auto request = Request::make_request<SetTimeout>(
      hardware_parameter.can_id.input, hardware_parameter.timeout_ms);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetInvertedEncoder>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_inverted);
    communication_node->sendRequest(std::move(request), 100ms);
  }  
  {
    if (parameter.closed_loop) {
      auto request = Request::make_request<SetClosedLoop>(hardware_parameter.can_id.input);
      communication_node->sendRequest(std::move(request), 100ms);
    }
    else {
      auto request = Request::make_request<SetOpenLoop>(hardware_parameter.can_id.input);
      communication_node->sendRequest(std::move(request), 100ms);    
    }
  }
  {
    auto request = Request::make_request<SetFrequency>(
      hardware_parameter.can_id.input, hardware_parameter.control_frequency);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetCtlKp>(hardware_parameter.can_id.input, parameter.kp);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetCtlKi>(hardware_parameter.can_id.input, parameter.ki);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetCtlKd>(hardware_parameter.can_id.input, parameter.kd);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetCtlAntiWindUp>(hardware_parameter.can_id.input, true);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetCtlInputFilter>(
      hardware_parameter.can_id.input, hardware_parameter.weight_low_pass_set_point);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetGearRatio>(
      hardware_parameter.can_id.input, hardware_parameter.gear_ratio);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetTicksPerRevision>(
      hardware_parameter.can_id.input, hardware_parameter.encoder_ratio);
    communication_node->sendRequest(std::move(request), 100ms);
  }
  {
    auto request = Request::make_request<SetRpmMax>(hardware_parameter.can_id.input, parameter.max_rpm);
    communication_node->sendRequest(std::move(request), 100ms);
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

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  initialize_controller(parameter, _parameter, _communication_node);

  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware::processSending, this), 50ms
  );
  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, Response>(
    _parameter.can_id.output,
    std::bind(&MotorControllerHardware::processRxData, this, std::placeholders::_1)
  );
}

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 2) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  std::scoped_lock lock(_data.mutex);
  _data.rpm = rpm;
}

void MotorControllerHardware::processSending()
{
  _data.mutex.lock();
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
