/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/motor_controller.hpp>

#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_device_interfaces.hpp"
#include "edu_robot/rpm.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

template <std::size_t NUM_CHANNELS>
class MotorControllerHardware : public MotorController::HardwareInterface
                              , public EthernetGatewayTxRxDevice
{
public:
  MotorControllerHardware(const std::uint8_t can_id, std::shared_ptr<EthernetCommunicator> communicator)
    : MotorController::HardwareInterface(NUM_CHANNELS)
    , EthernetGatewayTxRxDevice(communicator)
    , _can_id(can_id)
    , _measured_rpm(NUM_CHANNELS, 0.0)
  { }
  ~MotorControllerHardware() override = default;

  void processSetValue(const std::vector<Rpm>& rpm) override;
  void initialize(const Motor::Parameter& parameter) override;
  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;

private:
  std::uint8_t _can_id;
  std::vector<Rpm> _measured_rpm;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
