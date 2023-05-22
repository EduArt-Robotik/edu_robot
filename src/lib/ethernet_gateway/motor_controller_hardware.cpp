#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"
#include "edu_robot/rotation_per_minute.hpp"

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/hardware_error.hpp>

#include <exception>
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

SingleChannelMotorControllerHardware::SingleChannelMotorControllerHardware(
  const std::string& hardware_name_motor, const std::uint8_t can_id, std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name_motor)
  , EthernetGatewayTxRxDevice(hardware_name_motor, communicator)
  , _can_id(can_id)
{ }

SingleChannelMotorControllerHardware::~SingleChannelMotorControllerHardware()
{ }

void SingleChannelMotorControllerHardware::processRxData(const tcp::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || tcp::message::RpmMeasurement::canId(data) != _can_id) {
    return;
  }
  
  _callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
}

void SingleChannelMotorControllerHardware::initialize(const MotorController::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
  {
    auto request = Request::make_request<SetMotorControllerParameter>(
      0,
      _can_id,
      parameter.gear_ratio,
      parameter.max_rpm,
      parameter.threshold_stall_check,
      parameter.weight_low_pass_set_point,
      parameter.control_frequency
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Motor Controller Parameter\" was not acknowledged.");
    }
  }
  // Encoder Parameter
  {
    auto request = Request::make_request<SetEncoderParameter>(
      0,
      _can_id,
      parameter.encoder_ratio,
      parameter.weight_low_pass_encoder,
      parameter.encoder_inverted
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Encoder Parameter\" was not acknowledged.");
    }
  }
  // Pid Controller
  {
    auto request = Request::make_request<SetPidControllerParameter>(
      0,
      _can_id,
      parameter.kp,
      parameter.ki,
      parameter.kd,
     -parameter.max_rpm,
      parameter.max_rpm,
      parameter.weight_low_pass_set_point,
      true
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Pid Controller Parameter\" was not acknowledged.");
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
  //     throw std::runtime_error("Request \"Set Motor RPM Measurement\" was not acknowledged.");
  //   }
  // }  
}

void SingleChannelMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  auto request = Request::make_request<tcp::message::SetMotorRpm>(
    _can_id,
    rpm,
    Rpm(0.0)
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();

  if (_callback_process_measurement == nullptr) {
    return;
  }
  auto response = got.response();
  // \todo process feedback
  // _callback_process_measurement(AcknowledgedMotorRpm::rpm0(response));  
  _callback_process_measurement(0.0);
}


DummyMotorControllerHardware::DummyMotorControllerHardware(const std::string& hardware_name)
  : _current_set_value(0.0)
{
  (void)hardware_name; // \todo do something with hardware name or just remove it!
}

DummyMotorControllerHardware::~DummyMotorControllerHardware()
{

}

void DummyMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  _current_set_value = rpm;
}

void DummyMotorControllerHardware::initialize(const MotorController::Parameter& parameter)
{
  (void)parameter;
}

CompoundMotorControllerHardware::CompoundMotorControllerHardware(const std::string& hardware_name_motor_a,
                                                                 const std::string& hardware_name_motor_b,
                                                                 const std::uint8_t can_id,
                                                                 std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name_motor_b)
  , EthernetGatewayTxRxDevice(hardware_name_motor_b, communicator)
  , _can_id(can_id)
{
  _dummy_motor_controller = std::make_shared<DummyMotorControllerHardware>(hardware_name_motor_a);
}

CompoundMotorControllerHardware::~CompoundMotorControllerHardware()
{

}

void CompoundMotorControllerHardware::processRxData(const tcp::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || tcp::message::RpmMeasurement::canId(data) != _can_id) {
    return;
  }
  
  _dummy_motor_controller->_callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
  _callback_process_measurement(tcp::message::RpmMeasurement::rpm1(data));
}

void CompoundMotorControllerHardware::initialize(const MotorController::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
  {
    auto request = Request::make_request<SetMotorControllerParameter>(
      0,
      _can_id,
      parameter.gear_ratio,
      parameter.max_rpm,
      parameter.threshold_stall_check,
      parameter.weight_low_pass_set_point,
      parameter.control_frequency
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Motor Controller Parameter\" was not acknowledged.");
    }
  }
  // Encoder Parameter
  {
    auto request = Request::make_request<SetEncoderParameter>(
      0,
      _can_id,
      parameter.encoder_ratio,
      parameter.weight_low_pass_encoder,
      parameter.encoder_inverted
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Encoder Parameter\" was not acknowledged.");
    }
  }
  // Pid Controller
  {
    auto request = Request::make_request<SetPidControllerParameter>(
      0,
      _can_id,
      parameter.kp,
      parameter.ki,
      parameter.kd,
     -parameter.max_rpm,
      parameter.max_rpm,
      parameter.weight_low_pass_set_point,
      true
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 200ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Pid Controller Parameter\" was not acknowledged.");
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
  //     throw std::runtime_error("Request \"Set Motor RPM Measurement\" was not acknowledged.");
  //   }
  // }  
}

void CompoundMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  auto request = Request::make_request<tcp::message::SetMotorRpm>(
    _can_id,
    _dummy_motor_controller->_current_set_value,
    rpm
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _dummy_motor_controller->_callback_process_measurement(AcknowledgedMotorRpm::rpm0(got.response()));
  _callback_process_measurement(AcknowledgedMotorRpm::rpm1(got.response()));  
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
