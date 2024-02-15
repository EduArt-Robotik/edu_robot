#include "edu_robot/hardware/can/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can/message_definition.hpp"
#include "edu_robot/hardware/can/can_request.hpp"
#include "edu_robot/hardware/can/can_rx_data_endpoint.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>

#include <edu_robot/hardware_error.hpp>

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

using namespace std::chrono_literals;

using message::motor_controller::Enable;
using message::motor_controller::Disable;
using message::motor_controller::SetTimeout;
using message::motor_controller::SetInvertedEncoder;
using message::motor_controller::SetPwm;
using message::motor_controller::SetRpm;
using message::motor_controller::SetOpenLoop;
using message::motor_controller::SetClosedLoop;
using message::motor_controller::SetFrequency;
using message::motor_controller::SetCtlKp;
using message::motor_controller::SetCtlKi;
using message::motor_controller::SetCtlKd;
using message::motor_controller::SetCtlAntiWindUp;
using message::motor_controller::SetCtlInputFilter;
using message::motor_controller::SetGearRatio;
using message::motor_controller::SetTicksPerRevision;
using message::motor_controller::SetRpmMax;
using message::motor_controller::Response;

void initialize_controller(
  const Motor::Parameter& parameter, const std::uint8_t can_id, std::shared_ptr<Communicator> communicator)
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
  if (_callback_process_measurement == nullptr || Response::canId(data) != _can_id.output) {
    return;
  }
  
  _measured_rpm[0] = Response::rpm0(data);
  _measured_rpm[1] = Response::rpm1(data);  
  _callback_process_measurement(_measured_rpm, Response::enabled(data));
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  initialize_controller(parameter, _can_id.input, _communicator);

  auto measurement_end_point = CanRxDataEndPoint::make_data_endpoint<Response>(
    _can_id.output,
    std::bind(&MotorControllerHardware::processRxData, this, std::placeholders::_1)
  );
  _communicator->registerRxDataEndpoint(std::move(measurement_end_point));  
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
    _can_id.input,
    rpm[0],
    rpm[1]
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
