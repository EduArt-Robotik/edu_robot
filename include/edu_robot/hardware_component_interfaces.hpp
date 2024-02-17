/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <functional>

namespace eduart {
namespace robot {

template <typename Parameter>
class HardwareComponent
{
protected:
  virtual ~HardwareComponent() = default;
public:
  virtual void initialize(const Parameter& parameter) = 0;
};

template <typename... Data>
class HardwareActuator
{
public:
  virtual ~HardwareActuator() = default;
  virtual void processSetValue(const Data&... values) = 0;
};

template <typename... Data>
class HardwareSensor
{
public:
  using ProcessMeasurementCallback = std::function<void(const Data&...)>;

  virtual ~HardwareSensor() = default;
  void registerCallbackProcessMeasurementData(ProcessMeasurementCallback callback) {
    _callback_process_measurement = callback;
  }

protected:
  ProcessMeasurementCallback _callback_process_measurement;
};

} // end namespace eduart
} // end namespace robot
