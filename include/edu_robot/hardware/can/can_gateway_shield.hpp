/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/communicator.hpp"

#include <edu_robot/hardware_robot_interface.hpp>
#include <edu_robot/robot_status_report.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

class CanGatewayShield : public HardwareRobotInterface
                       , public processing::ProcessingComponentOutput<float>
{
public:
  CanGatewayShield(char const* const can_device);
  ~CanGatewayShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<Communicator> getCommunicator() { return _communicator; }

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  std::shared_ptr<Communicator> _communicator;
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

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
