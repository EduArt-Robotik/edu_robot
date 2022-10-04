#include "edu_robot/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/motor_controller.hpp"

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

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



CompoundMotorControllerHardware::CompoundMotorControllerHardware(const std::string& hardware_name,
                                                                 const eduart::robot::MotorController::Parameter& parameter,
                                                                 std::shared_ptr<IotShieldCommunicator> communicator)
  : IotShieldDevice(hardware_name + "_d")
  , IotShieldTxRxDevice(hardware_name + "_d", communicator)
{
  _dummy_motor_controllers[0] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_a");
  _dummy_motor_controllers[1] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_b");
  _dummy_motor_controllers[2] = std::make_shared<DummyMotorControllerHardware>(hardware_name + "_c");

  // Initial Motor Controller Hardware
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  using uart::message::UART;

  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::KP>(parameter.kp).data());
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::KI>(parameter.ki).data());
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::KD>(parameter.kd).data());
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::SET_POINT_LOW_PASS>(
    parameter.weight_low_pass_set_point).data()
  );
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::ENCODER_LOW_PASS>(
    parameter.weight_low_pass_encoder).data()
  );
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::GEAR_RATIO>(
    parameter.gear_ratio).data()
  );
  _communicator->sendBytes(uart::message::SetValueF<UART::COMMAND::SET::TICKS_PER_REV>(
    parameter.encoder_ratio).data()
  );
  _communicator->sendBytes(uart::message::SetValueU<UART::COMMAND::SET::CONTROL_FREQUENCY>(
    parameter.control_frequency).data()
  );
}            

CompoundMotorControllerHardware::~CompoundMotorControllerHardware()
{

}

void CompoundMotorControllerHardware::processRxData(const uart::message::RxMessageDataBuffer &data)
{
  if (_callback_process_measurement == nullptr) {
    return;
  }
  
  const uart::message::ShieldResponse msg(data);

  _dummy_motor_controllers[0]->_callback_process_measurement(msg.rpm0());
  _dummy_motor_controllers[1]->_callback_process_measurement(msg.rpm1());
  _dummy_motor_controllers[2]->_callback_process_measurement(msg.rpm2());
  _callback_process_measurement(msg.rpm3());
}

void CompoundMotorControllerHardware::processSetValue(const Rpm& rpm)
{
  const uart::message::SetRpm uart_message(
    _dummy_motor_controllers[0]->_current_set_value,
    _dummy_motor_controllers[1]->_current_set_value,
    _dummy_motor_controllers[2]->_current_set_value,
    rpm
  );

  _communicator->sendBytes(uart_message.data());
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
