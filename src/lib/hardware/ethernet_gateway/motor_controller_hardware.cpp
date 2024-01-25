#include "edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/tcp/protocol.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>

#include <edu_robot/hardware_error.hpp>

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

using tcp::message::PROTOCOL;
using tcp::message::Acknowledgement;
using tcp::message::SetEncoderParameter;
using tcp::message::SetMotorControllerParameter;
using tcp::message::SetPidControllerParameter;
using tcp::message::SetMotorRpm;
using tcp::message::SetMotorMeasurement;
using tcp::message::AcknowledgedMotorRpm;

void initialize_controller(
  const Motor::Parameter& parameter, const std::uint8_t can_id, std::shared_ptr<EthernetCommunicator> communicator)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
  {
    auto request = Request::make_request<SetMotorControllerParameter>(
      0,
      can_id,
      parameter.gear_ratio,
      parameter.max_rpm,
      parameter.threshold_stall_check,
      parameter.weight_low_pass_set_point,
      parameter.control_frequency,
      parameter.timeout_ms
    );
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw ComponentError(State::MOTOR_ERROR, "Request \"Set Motor Controller Parameter\" was not acknowledged.");
    }
  }
  // Encoder Parameter
  {
    auto request = Request::make_request<SetEncoderParameter>(
      0,
      can_id,
      parameter.encoder_ratio,
      parameter.weight_low_pass_encoder,
      parameter.encoder_inverted
    );
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw ComponentError(State::MOTOR_ERROR, "Request \"Set Encoder Parameter\" was not acknowledged.");
    }
  }
  // Pid Controller
  {
    auto request = Request::make_request<SetPidControllerParameter>(
      0,
      can_id,
      parameter.kp,
      parameter.ki,
      parameter.kd,
     -parameter.max_rpm,
      parameter.max_rpm,
      parameter.weight_low_pass_set_point,
      true
    );
    auto future_response = communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw ComponentError(State::MOTOR_ERROR, "Request \"Set Pid Controller Parameter\" was not acknowledged.");
    }
  }
  // Enable RPM Measurement Feedback
  // Note: disabled for the moment.
  // {
  //   auto request = Request::make_request<SetMotorMeasurement>(true);
  //   auto future_response = _communicator->sendRequest(std::move(request));
  //   wait_for_future(future_response, 200ms);

  //   auto got = future_response.get();
  //   if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_MEASUREMENT>::wasAcknowledged(got.response()) == false) {
  //     throw ComponentError("Request \"Set Motor RPM Measurement\" was not acknowledged.");
  //   }
  // }  
}

template <>
void MotorControllerHardware<1>::processRxData(const tcp::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || tcp::message::RpmMeasurement::canId(data) != _can_id) {
    return;
  }
  
  // \todo this processing is not ready on gateway side  
  // _dummy_motor_controller->_callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
  // _callback_process_measurement(tcp::message::RpmMeasurement::rpm1(data));
}

template <>
void MotorControllerHardware<2>::processRxData(const tcp::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || tcp::message::RpmMeasurement::canId(data) != _can_id) {
    return;
  }
  
  // \todo this processing is not ready on gateway side  
  // _dummy_motor_controller->_callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
  // _callback_process_measurement(tcp::message::RpmMeasurement::rpm1(data));
}

template <>
void MotorControllerHardware<1>::initialize(const Motor::Parameter& parameter)
{
  initialize_controller(parameter, _can_id, _communicator);
}

template <>
void MotorControllerHardware<2>::initialize(const Motor::Parameter& parameter)
{
  initialize_controller(parameter, _can_id, _communicator);
}

template <>
void MotorControllerHardware<1>::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 1) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  auto request = Request::make_request<tcp::message::SetMotorRpm>(
    _can_id,
    rpm[0],
    0.0
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
  _measured_rpm[0] = AcknowledgedMotorRpm::rpm0(got.response());
  _measured_rpm[1] = 0.0;  

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(_measured_rpm, AcknowledgedMotorRpm::enabled(got.response()));
}

template <>
void MotorControllerHardware<2>::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 2) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  auto request = Request::make_request<tcp::message::SetMotorRpm>(
    _can_id,
    rpm[0],
    rpm[1]
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
  _measured_rpm[0] = AcknowledgedMotorRpm::rpm0(got.response());
  _measured_rpm[1] = AcknowledgedMotorRpm::rpm1(got.response());  

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(_measured_rpm, AcknowledgedMotorRpm::enabled(got.response()));
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
