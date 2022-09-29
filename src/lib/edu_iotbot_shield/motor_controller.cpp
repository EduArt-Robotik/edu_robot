#include "edu_robot/iot_shield/motor_controller.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
#include "edu_robot/motor_controller.hpp"

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace iotbot {

DummyMotorController::DummyMotorController(const std::string& name, const std::uint8_t id,
                                           const eduart::robot::MotorController::Parameter& parameter,
                                           const std::string& urdf_joint_name, rclcpp::Node& ros_node)
  : eduart::robot::MotorController(name, id, parameter, urdf_joint_name, ros_node)
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



CompoundMotorController::CompoundMotorController(const std::string& name, std::shared_ptr<IotShieldCommunicator> communicator,
                                                 const eduart::robot::MotorController::Parameter& parameter,
                                                 rclcpp::Node& ros_node)
  : IotShieldDevice(name, 3u)
  , eduart::robot::MotorController(name + "_d", 3u, parameter, "base_to_wheel_rear_right", ros_node)
  , IotShieldTxRxDevice(name + "_d", 3u, communicator)
{
  _dummy_motor_controllers[0] = std::make_shared<DummyMotorController>(
    name + "_a",
    0u,
    parameter,
    "base_to_wheel_front_left",
    ros_node
  );
  _dummy_motor_controllers[1] = std::make_shared<DummyMotorController>(
    name + "_b",
    1u,
    parameter,
    "base_to_wheel_front_right",
    ros_node
  );
  _dummy_motor_controllers[2] = std::make_shared<DummyMotorController>(
    name + "_c",
    2u,
    parameter,
    "base_to_wheel_rear_left",
    ros_node
  );

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

void CompoundMotorController::processRxData(const uart::message::RxMessageDataBuffer &data)
{
  const uart::message::ShieldResponse msg(data);

  _dummy_motor_controllers[0]->processMeasurementData(msg.rpm0());
  _dummy_motor_controllers[1]->processMeasurementData(msg.rpm1());
  _dummy_motor_controllers[2]->processMeasurementData(msg.rpm2());
  processMeasurementData(msg.rpm3());
}

void CompoundMotorController::processSetRpm(const Rpm rpm)
{
  const uart::message::SetRpm uart_message(
    _dummy_motor_controllers[0]->_set_rpm,
    _dummy_motor_controllers[1]->_set_rpm,
    _dummy_motor_controllers[2]->_set_rpm,
    rpm
  );

  _communicator->sendBytes(uart_message.data());
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
