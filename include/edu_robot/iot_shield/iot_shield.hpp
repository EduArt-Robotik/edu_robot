/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"

#include <edu_robot/hardware_robot_interface.hpp>
#include <edu_robot/robot_status_report.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <memory>
#include <vector>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldCommunicator;
class IotShieldRxDevice;

class IotShield : public HardwareRobotInterface
                , public processing::ProcessingComponentOutput<float>
{
public:
  IotShield(char const* const device_name);
  ~IotShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<IotShieldCommunicator> getCommunicator() { return _communicator; }
  void registerIotShieldRxDevice(std::shared_ptr<IotShieldRxDevice> device);
  void processStatusReport();

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  std::shared_ptr<IotShieldCommunicator> _communicator;
  uart::message::TxMessageDataBuffer _tx_buffer;
  std::vector<std::shared_ptr<IotShieldRxDevice>> _rx_devices;
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

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
