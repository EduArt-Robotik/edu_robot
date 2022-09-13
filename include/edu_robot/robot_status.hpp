#pragma once

#include "edu_robot/mode.hpp"
#include "edu_robot/status.hpp"

#include <string>

namespace eduart {
namespace robot {

struct RobotStatus {
  Status status;
  Mode mode;
  std::string info_message;
};

} // end namespace robot
} // end namespace eduart