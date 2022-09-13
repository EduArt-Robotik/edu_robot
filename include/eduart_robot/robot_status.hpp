#pragma once

#include "eduart_robot/mode.hpp"
#include "eduart_robot/status.hpp"

#include <string>

namespace eduart_robot {

struct RobotStatus {
  Status status;
  Mode mode;
  std::string info_message;
};

} // end namespace eduart_robot