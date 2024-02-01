#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"
#include "edu_robot/hardware/igus/can/can_request.hpp"
#include "edu_robot/hardware/igus/can/message_definition.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <edu_robot/component_error.hpp>
#include <edu_robot/rpm.hpp>
#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

using namespace std::chrono_literals;

using hardware::igus::can::CanRequest;
using can::message::PROTOCOL;
using can::message::SetVelocity;
using can::message::AcknowledgedVelocity;
using can::message::SetEnableMotor;
using can::message::SetDisableMotor;
using can::message::SetReset;

void MotorControllerHardware::processRxData(const can::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr || can::message::AcknowledgedVelocity::canId(data) != _can_id) {
    return;
  }
}

void MotorControllerHardware::initialize(const Motor::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
}

void MotorControllerHardware::processSetValue(const std::vector<Rpm>& rpm)
{
  if (rpm.size() < 1) {
    throw std::runtime_error("Given RPM vector is too small.");
  }

  auto request = CanRequest::make_request<SetVelocity>(
    _can_id,
    rpm[0],
    getTimeStamp()
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
  _measured_rpm[0] = AcknowledgedVelocity::rpm(got.response());

  if (_callback_process_measurement == nullptr) {
    return;
  }
  if (AcknowledgedVelocity::errorCode(got.response()) & ~PROTOCOL::ERROR::MOTOR_NOT_ENABLED) {
    // \todo do some error handling here
  }

  _callback_process_measurement(_measured_rpm, AcknowledgedVelocity::enabled(got.response()));
}

void MotorControllerHardware::enable()
{
  auto request = CanRequest::make_request<SetEnableMotor>(_can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}
  
void MotorControllerHardware::disable()
{
  auto request = CanRequest::make_request<SetDisableMotor>(_can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
}

void MotorControllerHardware::reset()
{
  auto request = CanRequest::make_request<SetReset>(_can_id);
  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 200ms);
  auto got = future_response.get();
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
