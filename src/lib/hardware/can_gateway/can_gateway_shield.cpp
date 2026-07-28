#include "edu_robot/hardware/can_gateway/can_gateway_shield.hpp"
#include "edu_robot/hardware/can_gateway/can_communication_device.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/can/message_definition.hpp"
#include "edu_robot/hardware/can_gateway/can/can_rx_data_endpoint.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"
#include "edu_robot/hardware/communicator_node.hpp"

#include <edu_robot/executer.hpp>
#include <edu_robot/event.hpp>

#include <rclcpp/logging.hpp>

#include <memory>
#include <mutex>
#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

using namespace std::chrono_literals;

using hardware::can_gateway::CanCommunicationDevice;
using hardware::can_gateway::can::CanRxDataEndPoint;
using hardware::can_gateway::can::Request;

CanGatewayShield::CanGatewayShield(char const* const can_device)
  : _communicator{
      std::make_shared<Communicator>(
        std::make_shared<CanCommunicationDevice>(can_device, CanCommunicationDevice::CanType::CAN), 1ms),
      nullptr,
      nullptr
    }
{
  // Output for Measurements
  createOutput<float>("system.voltage");
  createOutput<float>("system.current");
  createOutput<float>("system.temperature");
  createOutput<Event>("event");

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

  // Executer
  for (std::size_t i = 0; i < 4; ++i) {
    _executer.emplace_back(std::make_shared<Executer>());
    _executer.back()->start();
  }
  _communication_node = std::make_shared<CommunicatorNode>(_executer[0], _communicator[0]);
}

CanGatewayShield::CanGatewayShield(char const* const can_device_0, char const* const can_device_1, char const* const can_device_2)
  : CanGatewayShield(can_device_0)
{
  (void)can_device_1;
  (void)can_device_2;
  // These two communicators are not part of this rx node!
  //_communicator[1] = std::make_shared<Communicator>(
  //  std::make_shared<CanCommunicationDevice>(can_device_1, CanCommunicationDevice::CanType::CAN_FD), 1ms
  //);
  //_communicator[2] = std::make_shared<Communicator>(
  //  std::make_shared<CanCommunicationDevice>(can_device_2, CanCommunicationDevice::CanType::CAN_FD), 1ms
  //);

  // Creating Data Endpoints for Measurements
  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, can::message::power_management::Response>(
    0x580,
    std::bind(&CanGatewayShield::processPowerManagementBoardResponse, this, std::placeholders::_1)
  );
  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, can::message::power_management::ShutdownCommand>(
    0x580,
    [this](const message::RxMessageDataBuffer &){ 
      output("event")->setValue(Event::SHUTDOWN);
    }
  );
  _communication_node->createRxDataEndPoint<CanRxDataEndPoint, can::message::can_gateway_shield::Response>(
    0x381,
    std::bind(&CanGatewayShield::processCanGatewayShieldResponse, this, std::placeholders::_1)
  );

  initialize();
}

CanGatewayShield::~CanGatewayShield()
{
  disable();
  for (auto& executer : _executer) {
    executer->stop();
  }
}

void CanGatewayShield::initialize()
{
  try {
    auto request = Request::make_request_with_response<can::message::power_management::GetHardware>(
      0x500, 0x580);
    const auto got = _communication_node->sendRequest(std::move(request), 200ms);

    const auto major = can::message::power_management::Hardware::major(got.response());
    const auto minor = can::message::power_management::Hardware::minor(got.response());
    const auto patch = can::message::power_management::Hardware::patch(got.response());

    _hardware_version = {major, minor, patch};
    RCLCPP_INFO(rclcpp::get_logger("CanGatewayShield"), "Hardware version: %d.%d.%d", major, minor, patch);
  }
  catch (...) {
    // hardware version request failed --> fall back to 0.x.x hardware version
    _hardware_version = {0, 0, 0};
  }
}

void CanGatewayShield::enable()
{
  std::scoped_lock lock(_mutex);

  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->enable();
  }
}

void CanGatewayShield::disable()
{
  std::scoped_lock lock(_mutex);

  for (auto& motor_controller : _motor_controller_hardware) {
    motor_controller->disable();
  }
}

// is called by executer thread
void CanGatewayShield::processPowerManagementBoardResponse(const message::RxMessageDataBuffer &data)
{
  using can::message::power_management::Response;
  std::scoped_lock lock(_mutex);

  if (Response::hasCorrectLength(data) == false) {
    return;
  }
  if (Response::isCurrent(data)) {
    // process system current measurement
    _status_report.current.mcu = Response::value(data);
    output("system.current")->setValue(_status_report.current.mcu);
  }
  else if (Response::isVoltage(data)) {
    // process system voltage
    _status_report.voltage.mcu = Response::value(data);
    output("system.voltage")->setValue(_status_report.voltage.mcu);
  }
  else {
    throw HardwareError(State::CAN_SOCKET_ERROR, "wrong message received");
  }

  // process status byte
  if (_hardware_version[0] >= 1) {
    // only read emergency stop state at hardware version 1.x.x
    if (_emergency_stop_active == false && Response::isEmergencyStop(data)) {
      output("event")->setValue(Event::EMERGENCY_STOP_PRESSED);
      _emergency_stop_active = true;
    }
    else if (_emergency_stop_active && Response::isEmergencyStop(data) == false) {
      output("event")->setValue(Event::EMERGENCY_STOP_RELEASED);
      _emergency_stop_active = false;
    }
  }
  if (_battery_empty == false && Response::isBatteryEmpty(data)) {
    output("event")->setValue(Event::BATTERY_EMPTY);
    _battery_empty = true;
  }
  else if (_battery_empty && Response::isBatteryEmpty(data) == false) {
    _battery_empty = false;
  }

  // Do Diagnostics
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();

  _diagnostic.processing_dt->update(dt / 1000000);
  _diagnostic.last_processing = now;

  _diagnostic.voltage->update(_status_report.voltage.mcu);
  _diagnostic.current->update(_status_report.current.mcu);
}

// is called by executer thread
void CanGatewayShield::processCanGatewayShieldResponse(const message::RxMessageDataBuffer &data)
{
  using can::message::can_gateway_shield::Response;

  if (Response::hasCorrectLength(data) == false) {
    // wrong message
    return;
  }

  std::scoped_lock lock(_mutex);
  _status_report.temperature = Response::temperature(data);
  output("system.temperature")->setValue(_status_report.temperature);

  // Do Diagnostics
  const auto now = _clock->now();
  const std::uint64_t dt = (now - _diagnostic.last_processing).nanoseconds();

  _diagnostic.processing_dt->update(dt / 1000000);
  _diagnostic.last_processing = now;

  _diagnostic.temperature->update(_status_report.temperature);
}

// is called by the main thread
RobotStatusReport CanGatewayShield::getStatusReport()
{
  std::scoped_lock lock(_mutex);
  return _status_report;
}

void CanGatewayShield::registerMotorControllerHardware(
  std::shared_ptr<MotorControllerHardware> motor_controller_hardware)
{
  std::scoped_lock lock(_mutex);

  if (std::find(_motor_controller_hardware.begin(), _motor_controller_hardware.end(), motor_controller_hardware)
      != _motor_controller_hardware.end())
  {
    throw std::runtime_error("CanGatewayShield: given motor controller already exists.");
  }

  _motor_controller_hardware.push_back(motor_controller_hardware);
} 

// is called by main thread
diagnostic::Diagnostic CanGatewayShield::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;
  std::scoped_lock lock(_mutex);

  diagnostic.add(*_diagnostic.voltage);
  diagnostic.add(*_diagnostic.current);
  diagnostic.add(*_diagnostic.temperature);
  diagnostic.add(*_diagnostic.processing_dt);

  return diagnostic;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
