#include "edu_robot/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/hardware_error.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/motor_controller.hpp"

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

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

CompoundMotorControllerHardware::CompoundMotorControllerHardware(
  const std::string& hardware_name, std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name + "_d")
  , IotShieldTxRxDevice(hardware_name + "_d", communicator)
{
  _dummy_motor_controllers[0] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_a");
  _dummy_motor_controllers[1] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_b");
  _dummy_motor_controllers[2] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_c");
}            

CompoundMotorControllerHardware::~CompoundMotorControllerHardware()
{

}

void CompoundMotorControllerHardware::processRxData(const uart::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  _dummy_motor_controllers[0]->_callback_process_measurement(-uart::message::ShieldResponse::rpm0(data));
  _dummy_motor_controllers[1]->_callback_process_measurement(-uart::message::ShieldResponse::rpm1(data));
  _dummy_motor_controllers[2]->_callback_process_measurement(-uart::message::ShieldResponse::rpm2(data));
  _callback_process_measurement(-uart::message::ShieldResponse::rpm3(data));
}

void CompoundMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  auto request = ShieldRequest::make_request<uart::message::SetRpm>(
    -_dummy_motor_controllers[0]->_current_set_value,
    -_dummy_motor_controllers[1]->_current_set_value,
    -_dummy_motor_controllers[2]->_current_set_value,
    -rpm
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);
  future_response.get();
}

void CompoundMotorControllerHardware::initialize(const MotorController::Parameter& parameter)
{
  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  using uart::message::UART;

  auto request = ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KP>>(
    parameter.kp, 0);
  auto future_response = _communicator->sendRequest(std::move(request));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KI>>(parameter.ki, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::KD>>(parameter.kd, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::SET_POINT_LOW_PASS>>(
      parameter.weight_low_pass_set_point, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::ENCODER_LOW_PASS>>(
    parameter.weight_low_pass_encoder, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::GEAR_RATIO>>(parameter.gear_ratio, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::TICKS_PER_REV>>(
      parameter.encoder_ratio, 0));
  future_response.wait_for(100ms);
  future_response.get();

  future_response = _communicator->sendRequest(
    ShieldRequest::make_request<uart::message::SetValueF<UART::COMMAND::SET::CONTROL_FREQUENCY>>(
      parameter.control_frequency, 0));
  future_response.wait_for(100ms);
  future_response.get();
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
