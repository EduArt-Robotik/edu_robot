#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/ethernet_gateway/tcp/message_definition.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"
#include "edu_robot/hardware_error.hpp"

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



CompoundMotorControllerHardware::CompoundMotorControllerHardware(const std::string& hardware_name_motor_a,
                                                                 const std::string& hardware_name_motor_b,
                                                                 const eduart::robot::MotorController::Parameter& parameter,
                                                                 const std::uint8_t can_id,
                                                                 std::shared_ptr<EthernetCommunicator> communicator)
  : EthernetGatewayDevice(hardware_name_motor_b)
  , EthernetGatewayTxRxDevice(hardware_name_motor_b, communicator)
  , _can_id(can_id)
{
  _dummy_motor_controller = std::make_shared<DummyMotorControllerHardware>(hardware_name_motor_a);

  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // Motor Controller Parameter
  {
    auto request = Request::make_request<SetMotorControllerParameter>(
      0,
      parameter.gear_ratio,
      parameter.max_rpm,
      parameter.threshold_stall_check,
      parameter.weight_low_pass_set_point,
      32
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Motor Controller Parameter\" was not acknowledged.");
    }
  }
  // Encoder Parameter
  {
    auto request = Request::make_request<SetEncoderParameter>(
      0,
      parameter.encoder_ratio,
      parameter.weight_low_pass_encoder,
      parameter.encoder_inverted
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Encoder Parameter\" was not acknowledged.");
    }
  }
  // Pid Controller
  {
    auto request = Request::make_request<SetPidControllerParameter>(
      0,
      parameter.kp,
      parameter.ki,
      parameter.kd,
     -parameter.max_rpm,
      parameter.max_rpm,
      parameter.weight_low_pass_set_point,
      true
    );
    auto future_response = _communicator->sendRequest(std::move(request));
    wait_for_future(future_response, 100ms);

    auto got = future_response.get();
    if (Acknowledgement<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>::wasAcknowledged(got.response()) == false) {
      throw std::runtime_error("Request \"Set Pid Controller Parameter\" was not acknowledged.");
    }
  }
}            

CompoundMotorControllerHardware::~CompoundMotorControllerHardware()
{

}

void CompoundMotorControllerHardware::processRxData(const tcp::message::RxMessageDataBuffer &data)
{
  (void)data;

  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  // _dummy_motor_controllers[0]->_callback_process_measurement(uart::message::ShieldResponse::rpm0(data));
  // _dummy_motor_controllers[1]->_callback_process_measurement(uart::message::ShieldResponse::rpm1(data));
  // _dummy_motor_controllers[2]->_callback_process_measurement(uart::message::ShieldResponse::rpm2(data));
  // _callback_process_measurement(uart::message::ShieldResponse::rpm3(data));
}

void CompoundMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  if (_can_id != 0) {
    // for debugging
    return;
  }

  auto request = Request::make_request<tcp::message::SetMotorRpm>(
    _dummy_motor_controller->_current_set_value,
    rpm
  );

  auto future_response = _communicator->sendRequest(std::move(request));
  wait_for_future(future_response, 100ms);

  auto got = future_response.get();
  if (Acknowledgement<PROTOCOL::COMMAND::SET::MOTOR_RPM>::wasAcknowledged(got.response()) == false) {
    throw std::runtime_error("Request \"Set Pid Controller Parameter\" was not acknowledged.");
  }  
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
