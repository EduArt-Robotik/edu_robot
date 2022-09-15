#pragma once

#include "edu_robot/mode.hpp"
#include "edu_robot/state.hpp"

#include <string>

namespace eduart {
namespace robot {

struct RobotState {
  State state;
  Mode mode;
  std::string info_message;
};

} // end namespace robot
} // end namespace eduart