/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware_robot_interface.hpp>

#include <edu_robot/robot_status_report.hpp>
#include <edu_robot/executer.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {

class Communicator;

namespace ethernet {

class EthernetGatewayShield : public HardwareRobotInterface
                            , public processing::DataSourceComponent
{
public:
  EthernetGatewayShield(char const* const ip_address, const std::uint16_t port);
  ~EthernetGatewayShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  inline std::shared_ptr<Communicator> getCommunicator() { return _communicator; }
  inline std::shared_ptr<Executer> getExecuter() { return _executer; }

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  std::shared_ptr<Communicator> _communicator;
  std::shared_ptr<Executer> _executer;
  std::shared_ptr<CommunicatorNode> _communication_node;
  std::shared_ptr<rclcpp::Clock> _clock;

  // diagnostics
  struct {
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::less<float>>> voltage;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> current;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> temperature;
    std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::uint64_t, std::greater<std::uint64_t>>> processing_dt;
    rclcpp::Time last_processing;
  } _diagnostic;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
