/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <functional>

namespace eduart {
namespace robot {

template <typename... DataType>
class HardwareComponentInterface
{
public:
  virtual ~HardwareComponentInterface() = default;

  virtual void processSetValue(const DataType&... values) = 0;
};

template <typename DataType>
class HardwareSensorInterface
{
public:
  using ProcessMeasurementCallback = std::function<void(const DataType&)>;

  void registerCallbackProcessMeasurementData(ProcessMeasurementCallback callback) {
    _callback_process_measurement = callback;
  }

protected:
  ProcessMeasurementCallback _callback_process_measurement;
};

} // end namespace eduart
} // end namespace robot
