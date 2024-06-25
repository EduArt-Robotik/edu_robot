#include "edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/communicator_node.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message_definition.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/protocol.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_request.hpp"

#include <cstddef>
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
namespace ethernet {

using namespace std::chrono_literals;

using udp::message::PROTOCOL;
using udp::message::Acknowledgement;
using udp::message::SetEncoderParameter;
using udp::message::SetMotorControllerParameter;
using udp::message::SetPidControllerParameter;
using udp::message::SetMotorRpm;
using udp::message::SetMotorMeasurement;
using udp::message::AcknowledgedMotorRpm;

template <std::size_t NUM_CHANNELS>
void initialize_controller(
  const Motor::Parameter& parameter, const typename MotorControllerHardware<NUM_CHANNELS>::Parameter& hardware_parameter,
  std::shared_ptr<CommunicatorNode> communicator_node)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
  {
    auto request = EthernetRequest::make_request<SetMotorControllerParameter>(
      0,
      hardware_parameter.can_id,
      hardware_parameter.gear_ratio,
      parameter.max_rpm,
      hardware_parameter.threshold_stall_check,
      hardware_parameter.weight_low_pass_set_point,
      hardware_parameter.control_frequency,
      hardware_parameter.timeout.count()
    );
    const auto got = communicator_node->sendRequest(std::move(request), 200ms);

    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw ComponentError(State::MOTOR_ERROR, "Request \"Set Motor Controller Parameter\" was not acknowledged.");
    }
  }
  // Encoder Parameter
  {
    auto request = EthernetRequest::make_request<SetEncoderParameter>(
      0,
      hardware_parameter.can_id,
      hardware_parameter.encoder_ratio,
      hardware_parameter.weight_low_pass_encoder,
      hardware_parameter.encoder_inverted
    );
    const auto got = communicator_node->sendRequest(std::move(request), 200ms);

    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw ComponentError(State::MOTOR_ERROR, "Request \"Set Encoder Parameter\" was not acknowledged.");
    }
  }
  // Pid Controller
  {
    auto request = EthernetRequest::make_request<SetPidControllerParameter>(
      0,
      hardware_parameter.can_id,
      parameter.kp,
      parameter.ki,
      parameter.kd,
     -parameter.max_rpm,
      parameter.max_rpm,
      hardware_parameter.weight_low_pass_set_point,
      true
    );
    const auto got = communicator_node->sendRequest(std::move(request), 200ms);

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
void MotorControllerHardware<1>::processRxData(const message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr ||
      udp::message::RpmMeasurement::canId(data) != _parameter.can_id) {
    return;
  }
  
  // \todo this processing is not ready on gateway side  
  // _dummy_motor_controller->_callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
  // _callback_process_measurement(tcp::message::RpmMeasurement::rpm1(data));
}

template <>
void MotorControllerHardware<2>::processRxData(const message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr ||
      udp::message::RpmMeasurement::canId(data) != _parameter.can_id) {
    return;
  }
  
  // \todo this processing is not ready on gateway side  
  // _dummy_motor_controller->_callback_process_measurement(tcp::message::RpmMeasurement::rpm0(data));
  // _callback_process_measurement(tcp::message::RpmMeasurement::rpm1(data));
}

// is called by the executer thread
template <>
void MotorControllerHardware<1>::processSending()
{
  const auto stamp_now = std::chrono::system_clock::now();
  _data.mutex.lock();

  if (_data.timeout == false && stamp_now - _data.stamp_last_rpm_set > _parameter.timeout) {
    // timeout occurred --> reset rpm values to zero
    std::fill(_data.rpm.begin(), _data.rpm.end(), 0.0f);
    _data.timeout = true;
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "timeout occurred! --> disable motor controller.");

    auto request = EthernetRequest::make_request<udp::message::SetMotorDisabled>();
    _communication_node->sendRequest(std::move(request), 200ms);
  }

  auto request = EthernetRequest::make_request<udp::message::SetMotorRpm>(
    _parameter.can_id,
    _data.rpm[0],
    0.0f
  );
  _data.mutex.unlock();

  const auto got = _communication_node->sendRequest(std::move(request), 200ms);
  _data.measured_rpm[0] = AcknowledgedMotorRpm::rpm0(got.response());
  _data.measured_rpm[1] = 0.0;  

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(_data.measured_rpm, AcknowledgedMotorRpm::enabled(got.response()));
}

template <>
void MotorControllerHardware<2>::processSending()
{
  const auto stamp_now = std::chrono::system_clock::now();
  _data.mutex.lock();

  if (_data.timeout == false && stamp_now - _data.stamp_last_rpm_set > _parameter.timeout) {
    // timeout occurred --> reset rpm values to zero
    std::fill(_data.rpm.begin(), _data.rpm.end(), 0.0f);
    _data.timeout = true;
    RCLCPP_INFO(rclcpp::get_logger("MotorControllerHardware"), "timeout occurred! --> disable motor controller.");

    auto request = EthernetRequest::make_request<udp::message::SetMotorDisabled>();
    _communication_node->sendRequest(std::move(request), 200ms);
  }

  auto request = EthernetRequest::make_request<udp::message::SetMotorRpm>(
    _parameter.can_id,
    _data.rpm[0],
    _data.rpm[1]
  );
  _data.mutex.unlock();

  const auto got = _communication_node->sendRequest(std::move(request), 200ms);
  _data.measured_rpm[0] = AcknowledgedMotorRpm::rpm0(got.response());
  _data.measured_rpm[1] = AcknowledgedMotorRpm::rpm1(got.response());  

  if (_callback_process_measurement == nullptr) {
    return;
  }

  _callback_process_measurement(_data.measured_rpm, AcknowledgedMotorRpm::enabled(got.response()));  
}

template <>
void MotorControllerHardware<1>::initialize(const Motor::Parameter& parameter)
{
  initialize_controller<1>(parameter, _parameter, _communication_node);
  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware<1>::processSending, this), 50ms
  );
}

template <>
void MotorControllerHardware<2>::initialize(const Motor::Parameter& parameter)
{
  initialize_controller<2>(parameter, _parameter, _communication_node);
  _communication_node->addSendingJob(
    std::bind(&MotorControllerHardware<2>::processSending, this), 50ms
  );  
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
