/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"

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

  DummyMotorController(const std::string& name, const std::uint8_t id, const eduart::robot::MotorController::Parameter& parameter,
                       const std::string& urdf_joint_name, rclcpp::Node& ros_node);
  ~DummyMotorController() override;

  void initialize(const eduart::robot::MotorController::Parameter& parameter) override;

private:
  void processSetRpm(const Rpm rpm) override;
};

class CompoundMotorController : public eduart::robot::MotorController
                              , public eduart::robot::iotbot::IotShieldTxRxDevice
{
public:
  CompoundMotorController(const std::string& name, std::shared_ptr<IotShieldCommunicator> communicator,
                          const eduart::robot::MotorController::Parameter& parameter, rclcpp::Node& ros_node);
  ~CompoundMotorController() override;

  void initialize(const eduart::robot::MotorController::Parameter& parameter) override;
  void processRxData(const uart::message::RxMessageDataBuffer& data) override;

  inline const std::array<std::shared_ptr<iotbot::DummyMotorController>, 3u>& dummyMotorController() const {
    return _dummy_motor_controllers;
  }

private:
  void processSetRpm(const Rpm rpm) override;

  std::array<std::shared_ptr<iotbot::DummyMotorController>, 3u> _dummy_motor_controllers;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
