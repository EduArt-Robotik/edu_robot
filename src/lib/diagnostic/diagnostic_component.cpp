#include "edu_robot/diagnostic/diagnostic_component.hpp"

namespace eduart {
namespace robot {
namespace diagnostic {

void DiagnosticComponent::processDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic)
{
  // process sensor specific diagnostics in derived class
  const diagnostic::Diagnostic sensor_diagnostic = processDiagnosticsImpl();

  // convert diagnostic in ROS diagnostic message
  diagnostic.summary(convert(sensor_diagnostic.level()), sensor_diagnostic.summery());

  for (const auto& entry : sensor_diagnostic.entries()) {
    diagnostic.add(entry.first, entry.second);
  }
}

} // end namespace diagnostic
} // end namespace robot
} // end namespace eduart
