/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communicator.hpp>
#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware_robot_interface.hpp>

#include <edu_robot/executer.hpp>
#include <edu_robot/robot_status_report.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <cstdint>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class MotorControllerHardware;

class CanGatewayShield : public HardwareRobotInterface
                       , public processing::ProcessingComponentOutput<float>
                       , public std::enable_shared_from_this<CanGatewayShield>
{
public:
  CanGatewayShield(char const* const can_device);
  CanGatewayShield(char const* const can_device_0, char const* const can_device_1, char const* const can_device_2);
  ~CanGatewayShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<Communicator> getCommunicator(const std::size_t index) {
    if (index >= _communicator.size()) {
      throw std::invalid_argument("CanGatewayShield::getCommunicator() given index is out of range.");
    }
    
    return _communicator[index];
  }
  inline std::shared_ptr<Executer> getExecuter() { return _executer; }
  void registerMotorControllerHardware(std::shared_ptr<MotorControllerHardware> motor_controller_hardware);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;
  void processPowerManagementBoardResponse(const message::RxMessageDataBuffer &data);
  void processCanGatewayShieldResponse(const message::RxMessageDataBuffer &data);

  std::array<std::shared_ptr<Communicator>, 3> _communicator;
  std::shared_ptr<Executer> _executer;
  std::shared_ptr<CommunicatorNode> _communication_node;
  std::mutex _mutex;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::vector<std::shared_ptr<MotorControllerHardware>> _motor_controller_hardware;

  RobotStatusReport _status_report;

  // diagnostics
  struct {
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::less<float>>> voltage;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> current;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> temperature;
    std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::uint64_t, std::greater<std::uint64_t>>> processing_dt;
    rclcpp::Time last_processing;
  } _diagnostic;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
