#include "edu_robot/motor_controller.hpp"
#include "edu_robot/rpm.hpp"

#include <cstdint>
#include <functional>

#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {

MotorController::MotorController(
  const std::string& name, const std::size_t id, std::vector<Motor>&& motors, rclcpp::Node& ros_node,
    std::shared_ptr<HardwareInterface> hardware_interface)
  : _name(name)
  , _id(id)
  , _motor(motors)
  , _hardware_interface(hardware_interface)
  , _clock(ros_node.get_clock())
  , _stamp_last_measurement(_clock->now())
  , _last_processing(_clock->now())
  , _processing_dt_statistic(std::make_shared<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>>(
      "set rpm dt", "ms", 20, 200, 1000, 30, 100)
    )    
{
  _hardware_interface->registerCallbackProcessMeasurementData(
    std::bind(&MotorController::processMeasurementData, this, std::placeholders::_1, std::placeholders::_2)
  );
}

MotorController::~MotorController()
{

}

void MotorController::setRpm(const std::vector<Rpm>& rpm)
{
  // Do statistics for diagnostic
  const auto now = _clock->now();
  const auto dt = (now - _last_processing).nanoseconds();
  _processing_dt_statistic->update(dt / 1000000);
  _last_processing = now;

  // set rpm
  _hardware_interface->processSetValue(rpm);
}

void MotorController::processMeasurementData(const std::vector<Rpm>& rpm, const bool enabled_flag)
{
  std::lock_guard guard(_mutex_access_data);
  bool all_enabled = true;
  const auto now = _clock->now();

  for (std::size_t i = 0; i < _motor.size(); ++i) {
    _motor[i].processMeasurementData(rpm[i], enabled_flag, now);
    all_enabled &= _motor[i].isEnabled();
  }

  if (all_enabled) {
    _lost_enable = false;
  }
}

diagnostic::Diagnostic MotorController::processDiagnosticsImpl()
{
  diagnostic::Diagnostic diagnostic;

  // processing dt
  if ((_clock->now() - _last_processing).nanoseconds() / 1000000 > _processing_dt_statistic->checkerMean().levelError()) {
    diagnostic.add("set rpm", "timeout", diagnostic::Level::ERROR);
  }
  else {
    diagnostic.add(*_processing_dt_statistic);
  }

  // lost enable
  diagnostic.add(
    "lost enable", _lost_enable, _lost_enable ? diagnostic::Level::ERROR : diagnostic::Level::OK
  );

  return diagnostic;
}

} // end namespace robot
} // end namespace eduart
