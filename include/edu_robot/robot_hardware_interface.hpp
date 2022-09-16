/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/rotation_per_minute.hpp"
#include "edu_robot/lighting.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <vector>

namespace eduart {
namespace robot {

class RobotHardwareInterface
{
public:
  virtual ~RobotHardwareInterface() = default;

  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual RobotStatusReport getStatusReport() = 0;

  bool isStatusReportReady() const { return _status_report_ready; }
  void clearStatusReport() { _status_report_ready = false; _report = RobotStatusReport(); }

protected:
  RobotStatusReport _report;
  bool _status_report_ready = false;
};

} // end namespace eduart
} // end namespace robot