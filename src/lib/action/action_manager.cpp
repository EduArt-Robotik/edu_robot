#include "edu_robot/action/action_manager.hpp"

namespace eduart {
namespace robot {
namespace action {

void ActionManager::addAction(std::shared_ptr<Action> action)
{
  _actions.push_back(action);
}

void ActionManager::process()
{
  for (auto action = _actions.begin(); action != _actions.end();) {
    if ((*action)->isReady()) {
      (*action)->process();
      (*action)->keepAlive() ? ++action : action = _actions.erase(action);
    }
    else {
      ++action;
    }
  }
}

} // end namespace action
} // end namespace eduart
} // end namespace robot
