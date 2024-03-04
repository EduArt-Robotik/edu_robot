#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>

#include <edu_robot/hardware_error.hpp>

#include <memory>
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

  ros_node.declare_parameter<int>(name + ".can_id.input", default_parameter.can_id.input);
  ros_node.declare_parameter<int>(name + ".can_id.output", default_parameter.can_id.output);

  parameter.can_id.input = ros_node.get_parameter(name + ".can_id.input").as_int();
  parameter.can_id.output = ros_node.get_parameter(name + ".can_id.output").as_int();

  return parameter;
}

void initialize_controller(
  const Motor::Parameter& parameter, const std::uint32_t can_id, std::shared_ptr<Communicator> communicator)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  {
    auto request = Request::make_request<SetTimeout>(can_id, parameter.timeout_ms);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetInvertedEncoder>(can_id, parameter.encoder_inverted);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }  
  {
    if (parameter.closed_loop) {
      auto request = Request::make_request<SetClosedLoop>(can_id);
      auto future_response = communicator->sendRequest(std::move(request));
      wait_for_future(future_response, 100ms);
      auto got = future_response.get();
    }
    else {
      auto request = Request::make_request<SetOpenLoop>(can_id);
      auto future_response = communicator->sendRequest(std::move(request));
      wait_for_future(future_response, 100ms);
      auto got = future_response.get();      
    }
  }
  {
    auto request = Request::make_request<SetFrequency>(can_id, parameter.control_frequency);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetCtlKp>(can_id, parameter.kp);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetCtlKi>(can_id, parameter.ki);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetCtlKd>(can_id, parameter.kd);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetCtlAntiWindUp>(can_id, true);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetCtlInputFilter>(can_id, parameter.weight_low_pass_set_point);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetGearRatio>(can_id, parameter.gear_ratio);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetTicksPerRevision>(can_id, parameter.encoder_ratio);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
  {
    auto request = Request::make_request<SetRpmMax>(can_id, parameter.max_rpm);
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);
    auto got = future_response.get();
  }
}

// template <>
// void MotorControllerHardware<1>::processRxData(const message::RxMessageDataBuffer &data)
// {
//   if (_callback_process_measurement == nullptr || Response::canId(data) != _can_id.output) {
//     return;
//   }
  
//   _measured_rpm[0] = Response::rpm0(data);
//   _callback_process_measurement(_measured_rpm, Response::enabled(data));
// }

void MotorControllerHardware::processRxData(const message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || Response::canId(data) != _parameter.can_id.output) {
    return;
  }
  
  _measured_rpm[0] = Response::rpm0(data);
  _measured_rpm[1] = Response::rpm1(data);  
  _callback_process_measurement(_measured_rpm, Response::enabled(data));
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  initialize_controller(parameter, _parameter.can_id.input, _communicator);

  auto measurement_end_point = CanRxDataEndPoint::make_data_endpoint<Response>(
    _parameter.can_id.output,
    std::bind(&MotorControllerHardware::processRxData, this, std::placeholders::_1),
    this
  );
  _communicator->registerRxDataEndpoint(measurement_end_point);
  registerRxDataEndpoint(measurement_end_point);
}

// template <>
// void MotorControllerHardware<1>::processSetValue(const std::vector<Rpm>& rpm)
// {
//   if (rpm.size() < 1) {
//     throw std::runtime_error("Given RPM vector is too small.");
//   }

//   auto request = Request::make_request<SetRpm>(
//     _can_id.input,
//     rpm[0],
//     0.0
//   );

//   auto future_response = _communicator->sendRequest(std::move(request));
//   wait_for_future(future_response, 200ms);
//   auto got = future_response.get();
// }

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 2) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  auto request = Request::make_request<SetRpm>(
    _parameter.can_id.input,
    rpm[0],
    rpm[1]
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  auto got = future_response.get();
}

void MotorControllerHardware::enable()
{
  auto request = Request::make_request<Enable>(_parameter.can_id.input);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  auto got = future_response.get();  
}

void MotorControllerHardware::disable()
{
  auto request = Request::make_request<Disable>(_parameter.can_id.input);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  auto got = future_response.get();  
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
