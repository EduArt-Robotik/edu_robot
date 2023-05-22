/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <functional>
#include <type_traits>

namespace eduart {
namespace robot {

template <typename Parameter, typename... Data>
class HardwareComponentInterface
{
public:
  virtual ~HardwareComponentInterface() = default;

  virtual void processSetValue(const Data&... values) = 0;
  virtual void initialize(const Parameter& parameter) = 0;
};

template <typename Parameter, typename... Data>
class HardwareSensorInterface
{
public:
  using ProcessMeasurementCallback = std::function<void(const Data&...)>;

  virtual ~HardwareSensorInterface() = default;
  void registerCallbackProcessMeasurementData(ProcessMeasurementCallback callback) {
    _callback_process_measurement = callback;
  }

  virtual void initialize(const Parameter& parameter) = 0;

protected:
  ProcessMeasurementCallback _callback_process_measurement;
};

} // end namespace eduart
} // end namespace robot
