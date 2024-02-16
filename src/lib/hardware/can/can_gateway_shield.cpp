#include "edu_robot/hardware/can/can_gateway_shield.hpp"
#include "edu_robot/hardware/can/can_communication_device.hpp"
#include "edu_robot/hardware/can/motor_controller_hardware.hpp"

#include <memory>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can {

using namespace std::chrono_literals;

CanGatewayShield::CanGatewayShield(char const* const can_device)
  : processing::ProcessingComponentOutput<float>("can_gateway_shield")
{
  _communicator[0] = std::make_shared<Communicator>(std::make_shared<hardware::can::CanCommunicationDevice>(can_device), 10ms);

  // Configuring Diagnostic
  _clock = std::make_shared<rclcpp::Clock>();
  _diagnostic.voltage = std::make_shared<diagnostic::MeanDiagnostic<float, std::less<float>>>(
    "voltage", "V", 10, 18.7f, 18.0f
  );
  _diagnostic.current = std::make_shared<diagnostic::MeanDiagnostic<float, std::greater<float>>>(
    "current", "A", 10, 1.5f, 2.0f
  );
  _diagnostic.temperature = std::make_shared<diagnostic::MeanDiagnostic<float, std::greater<float>>>(
    "temperature", "°C", 10, 55.0f, 65.0f
  );
  _diagnostic.processing_dt = std::make_shared<diagnostic::StandardDeviationDiagnostic<std::uint64_t, std::greater<std::uint64_t>>>(
    "processing dt", "ms", 10, 700, 1000, 50, 100
  );
  _diagnostic.last_processing = _clock->now();
}

CanGatewayShield::CanGatewayShield(char const* const can_device_0, char const* const can_device_1, char const* const can_device_2)
  : CanGatewayShield(can_device_0)
{
  _communicator[1] = std::make_shared<Communicator>(std::make_shared<hardware::can::CanCommunicationDevice>(can_device_1), 10ms);
  _communicator[2] = std::make_shared<Communicator>(std::make_shared<hardware::can::CanCommunicationDevice>(can_device_2), 10ms);
}

CanGatewayShield::~CanGatewayShield()
{
  disable();
}

void CanGatewayShield::enable()
{
  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->enable();
  }
}

void CanGatewayShield::disable()
{
  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->disable();
  }
}

RobotStatusReport CanGatewayShield::getStatusReport()
{
  // Requesting Status Report
  // auto request = Request::make_request<tcp::message::GetStatus>();
  // auto future_response = _communicator->sendRequest(std::move(request));
  // wait_for_future(future_response, 200ms);
  // auto got = future_response.get();

  // Do Status Report
  RobotStatusReport report;

  // report.temperature = AcknowledgedStatus::temperature(got.response());
  // report.voltage.mcu = AcknowledgedStatus::voltage(got.response());
  // report.current.mcu = AcknowledgedStatus::current(got.response());
  // report.status_emergency_stop = AcknowledgedStatus::statusEmergencyButton(got.response());

  // sendInputValue(report.voltage.mcu);

  // // Do Diagnostics
  // const auto now = _clock->now();
  // const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();
  // _diagnostic.processing_dt->update(dt / 1000000);
  // _diagnostic.last_processing = now;

  // _diagnostic.voltage->update(report.voltage.mcu);
  // _diagnostic.current->update(report.current.mcu);
  // _diagnostic.temperature->update(report.temperature);

  return report;
}

void CanGatewayShield::registerMotorControllerHardware(
  std::shared_ptr<MotorControllerHardware> motor_controller_hardware)
{
  if (std::find(_motor_controller_hardware.begin(), _motor_controller_hardware.end(), motor_controller_hardware)
      != _motor_controller_hardware.end())
  {
    throw std::runtime_error("igus::CanGatewayShield: given motor controller already exists.");
  }

  _motor_controller_hardware.push_back(motor_controller_hardware);
} 

diagnostic::Diagnostic CanGatewayShield::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;

  diagnostic.add(*_diagnostic.voltage);
  diagnostic.add(*_diagnostic.current);
  diagnostic.add(*_diagnostic.temperature);
  diagnostic.add(*_diagnostic.processing_dt);

  return diagnostic;
}

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
