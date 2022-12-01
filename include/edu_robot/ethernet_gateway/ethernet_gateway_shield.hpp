/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include <edu_robot/robot_hardware_interface.hpp>
#include <edu_robot/robot_status_report.hpp>
#include <edu_robot/processing_component/processing_component.hpp>

#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class EthernetGatewayShield : public RobotHardwareInterface
                            , public processing::ProcessingComponentOutput<float>
{
public:
  EthernetGatewayShield(char const* const ip_address, const std::uint16_t port);
  ~EthernetGatewayShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<EthernetCommunicator> getCommunicator() { return _communicator; }

private:
  std::shared_ptr<EthernetCommunicator> _communicator;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
