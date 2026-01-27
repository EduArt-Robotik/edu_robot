/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/executer.hpp>

#include <edu_robot/hardware_robot_interface.hpp>
#include <edu_robot/robot_status_report.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

#include <edu_robot/processing_component/processing_component.hpp>

#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {

class Communicator;

namespace iot_shield {

class IotShieldRxDevice;

class IotShield : public HardwareRobotInterface
                , public processing::DataSourceComponent
{
public:
  struct Parameter {
    std::string uart_device = "/dev/ttyS1";
    bool via_tcp_connection = false;
    std::string tcp_host = "192.168.0.100";
    int tcp_port = 5000;
  };

  IotShield(const Parameter& parameter);
  IotShield(char const* const device_name);
  ~IotShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  inline std::shared_ptr<Communicator> getCommunicator() { return _communicator; }
  inline std::shared_ptr<Executer> getExecuter() { return _executer; }
  inline void setImuRawDataMode(const bool raw_data_mode) {
    _imu_raw_data_mode = raw_data_mode;
  }
  static Parameter get_parameter(
    const std::string& shield_name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void construct();
  void processStatusReport(const message::RxMessageDataBuffer& data);

  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<Communicator> _communicator;
  std::shared_ptr<Executer> _executer;
  std::shared_ptr<CommunicatorNode> _communication_node;
  std::shared_ptr<rclcpp::Clock> _clock;
  bool _imu_raw_data_mode = false;
  std::mutex _data_mutex;

  // diagnostics
  struct {
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::less<float>>> voltage;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> current;
    std::shared_ptr<diagnostic::MeanDiagnostic<float, std::greater<float>>> temperature;
    std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::uint64_t, std::greater<std::uint64_t>>> processing_dt;
    rclcpp::Time last_processing;
  } _diagnostic;  
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
