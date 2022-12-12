/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/rotation_per_minute.hpp"

#include <memory>
#include <string>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

class DummyMotorControllerHardware : public MotorController::ComponentInterface
                                   , public MotorController::SensorInterface
{
public:
  friend class CompoundMotorControllerHardware;

  DummyMotorControllerHardware(const std::string& hardware_name);
  ~DummyMotorControllerHardware() override;

  void processSetValue(const Rpm& rpm) override;
  void initialize(const MotorController::Parameter& parameter) override;

private:
  Rpm _current_set_value;
};

class CompoundMotorControllerHardware : public MotorController::ComponentInterface
                                      , public eduart::robot::iotbot::IotShieldTxRxDevice
                                      , public MotorController::SensorInterface
{
public:
  CompoundMotorControllerHardware(const std::string& name, std::shared_ptr<IotShieldCommunicator> communicator);
  ~CompoundMotorControllerHardware() override;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
  void processSetValue(const Rpm& rpm) override;
  void initialize(const MotorController::Parameter& parameter) override;

  inline const std::array<std::shared_ptr<DummyMotorControllerHardware>, 3u>& dummyMotorController() const {
    return _dummy_motor_controllers;
  }

private:
  std::array<std::shared_ptr<DummyMotorControllerHardware>, 3u> _dummy_motor_controllers;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
