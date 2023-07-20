/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/action/action.hpp"

#include <memory>
#include <list>

namespace eduart {
namespace robot {
namespace action {

class ActionManager
{
public:
  ActionManager() = default;

  /**
   * \brief This method needs be called for managing all registered actions.
   */
  void process();

  void addAction(std::shared_ptr<Action> action);

private:
  std::list<std::shared_ptr<Action>> _actions;
};

} // end namespace action
} // end namespace eduart
} // end namespace robot
