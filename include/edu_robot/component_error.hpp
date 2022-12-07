/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/state.hpp"

#include <stdexcept>

namespace eduart {
namespace robot {

/**
 * \brief Represents an hardware error that is occurred during runtime.
 */
class ComponentError : public std::runtime_error
{
public:
  ComponentError(const State error, const std::string& error_message)
    : std::runtime_error(error_message)
    , _error(error)
  { }

  State error() const { return _error; }

private:
  State _error;
};

} // end namespace robot
} // end namespace eduart
