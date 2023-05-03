/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"
#include "edu_robot/rotation_per_minute.hpp"

#include <memory>
#include <array>

namespace eduart {
namespace robot {
namespace ethernet {

class SingleChannelMotorControllerHardware : public MotorController::ComponentInterface
                                           , public EthernetGatewayTxRxDevice
                                           , public MotorController::SensorInterface
{
public:
  SingleChannelMotorControllerHardware(
    const std::string& hardware_name_motor, const std::uint8_t can_id, std::shared_ptr<EthernetCommunicator> communicator);
  ~SingleChannelMotorControllerHardware() override;

  void processSetValue(const Rpm& rpm) override;
  void initialize(const MotorController::Parameter& parameter) override;
  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;

private:
  std::uint8_t _can_id;
};

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
                                      , public EthernetGatewayTxRxDevice
                                      , public MotorController::SensorInterface
{
public:
  CompoundMotorControllerHardware(const std::string& hardware_name_motor_a,
                                  const std::string& hardware_name_motor_b,
                                  const std::uint8_t can_id,
                                  std::shared_ptr<EthernetCommunicator> communicator);
  ~CompoundMotorControllerHardware() override;

  void processSetValue(const Rpm& rpm) override;
  void initialize(const MotorController::Parameter& parameter) override;
  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;

  inline const std::shared_ptr<DummyMotorControllerHardware>& dummyMotorController() const {
    return _dummy_motor_controller;
  }

private:
  std::shared_ptr<DummyMotorControllerHardware> _dummy_motor_controller;
  std::uint8_t _can_id;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
