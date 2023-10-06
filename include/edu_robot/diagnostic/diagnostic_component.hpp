/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/diagnostic/diagnostic.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

namespace eduart {
namespace robot {
namespace diagnostic {

/**
 * \brief Base class for components that produce a diagnostic message. This class provides a generic formatting function
 *        for converting the diagnostic into a ROS diagnostic.
 */
class DiagnosticComponent
{
protected:
  DiagnosticComponent() = default;

  virtual diagnostic::Diagnostic processDiagnosticsImpl() = 0;

public:
  void processDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostics);
};

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
