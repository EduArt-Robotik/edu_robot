#include "edu_robot/iot_shield/motor_controller.hpp"
#include "edu_robot/iot_shield/uart_message_conversion.hpp"
#include "edu_robot/motor_controller.hpp"

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

DummyMotorController::DummyMotorController(const std::string& name, const std::uint8_t id, const eduart::robot::MotorController::Parameter& parameter)
  : eduart::robot::MotorController(name, id, parameter)
{

}

DummyMotorController::~DummyMotorController()
{

}

void DummyMotorController::initialize(const eduart::robot::MotorController::Parameter& /* parameter */)
{
  // Do nothing, only compound motor controller sends commands.
}

void DummyMotorController::processSetRpm(const Rpm /* rpm */)
{
  // Do nothing, only compound motor controller sends commands.
}



CompoundMotorController::CompoundMotorController(const std::string& name, const std::uint8_t id,
                                                 std::shared_ptr<IotShieldCommunicator> communicator,
                                                 const eduart::robot::MotorController::Parameter& parameter)
  : eduart::robot::MotorController(name + "_d", id, parameter)
  , IotShieldDevice(name + "_d", id, communicator)
{
  _dummy_motor_controllers[0] = std::make_shared<DummyMotorController>(name + "_a", 0u, parameter);
  _dummy_motor_controllers[1] = std::make_shared<DummyMotorController>(name + "_b", 1u, parameter);
  _dummy_motor_controllers[2] = std::make_shared<DummyMotorController>(name + "_c", 2u, parameter);

  initialize(parameter);
}            

CompoundMotorController::~CompoundMotorController()
{

}

void CompoundMotorController::initialize(const eduart::robot::MotorController::Parameter &parameter)
{
  if (false == parameter.isValid()) {
    throw std::invalid_argument("Given parameter are not valid. Cancel initialization of motor controller.");
  }

  // set control frequency
  {
    _tx_buffer = { 0 };
    _tx_buffer[0] = UART::BUFFER::START_BYTE;
    _tx_buffer[1] = UART::COMMAND::SET::CONTROL_FREQUENCY;
    uint32ToTxBuffer<2, 5>(parameter.control_frequency, _tx_buffer);
    _tx_buffer[10] = UART::BUFFER::END_BYTE;

    _communicator->sendBytes(_tx_buffer);
  }

  setValue<UART::COMMAND::SET::KP>(parameter.kp);
  setValue<UART::COMMAND::SET::KI>(parameter.ki);
  setValue<UART::COMMAND::SET::KD>(parameter.kd);
  setValue<UART::COMMAND::SET::SET_POINT_LOW_PASS>(parameter.weight_low_pass_set_point);
  setValue<UART::COMMAND::SET::ENCODER_LOW_PASS>(parameter.weight_low_pass_encoder);
  setValue<UART::COMMAND::SET::GEAR_RATIO>(parameter.gear_ratio);
  setValue<UART::COMMAND::SET::TICKS_PER_REV>(parameter.encoder_ratio);
}

void CompoundMotorController::processSetRpm(const Rpm rpm)
{
  _tx_buffer[0] = UART::BUFFER::START_BYTE;
  _tx_buffer[1] = UART::COMMAND::SET::RPM;

  int16ToTxBuffer<2, 3>(static_cast<std::int16_t>(_dummy_motor_controllers[0]->_set_rpm * 100.f + 0.5f), _tx_buffer);
  int16ToTxBuffer<4, 5>(static_cast<std::int16_t>(_dummy_motor_controllers[1]->_set_rpm * 100.f + 0.5f), _tx_buffer);
  int16ToTxBuffer<6, 7>(static_cast<std::int16_t>(_dummy_motor_controllers[2]->_set_rpm * 100.f + 0.5f), _tx_buffer);
  int16ToTxBuffer<8, 9>(static_cast<std::int16_t>(rpm * 100.f + 0.5f), _tx_buffer);

  _tx_buffer[10] = UART::BUFFER::END_BYTE;

  _communicator->sendBytes(_tx_buffer);
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
