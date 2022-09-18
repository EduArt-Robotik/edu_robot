/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/uart_message_conversion.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class DummyMotorController : public eduart::robot::MotorController
{
public:
  friend class CompoundMotorController;

  DummyMotorController(const std::string& name, const std::uint8_t id, const eduart::robot::MotorController::Parameter& parameter);
  ~DummyMotorController() override;

  void initialize(const eduart::robot::MotorController::Parameter& parameter) override;

private:
  void processSetRpm(const Rpm rpm) override;
};

class CompoundMotorController : public eduart::robot::MotorController
                              , public eduart::robot::iotbot::IotShieldDevice
{
public:
  CompoundMotorController(const std::string& name, const std::uint8_t id, std::shared_ptr<IotShieldCommunicator> communicator,
                          const eduart::robot::MotorController::Parameter& parameter);
  ~CompoundMotorController() override;

  void initialize(const eduart::robot::MotorController::Parameter& parameter) override;
  const std::array<std::shared_ptr<iotbot::DummyMotorController>, 3u>& dummyMotorController() const {
    return _dummy_motor_controllers;
  }

private:
  void processSetRpm(const Rpm rpm) override;

  template<std::uint8_t Command>
  void setValue(const float value)
  {
    // // clear buffer
    // _tx_buffer = { 0 };

    // // construct message
    // _tx_buffer[0] = UART::BUFFER::START_BYTE;
    // _tx_buffer[1] = Command;

    // floatToTxBuffer<2, 5>(value, _tx_buffer);
  
    // _tx_buffer[10] = UART::BUFFER::END_BYTE;

    // // send message
    // _communicator->sendBytes(_tx_buffer);    
  }

  std::array<std::shared_ptr<iotbot::DummyMotorController>, 3u> _dummy_motor_controllers;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
