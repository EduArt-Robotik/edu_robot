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
  for (auto action = _actions.begin(); action != _actions.end(); ++action) {
    if ((*action)->isReady()) {
      (*action)->process();
      _actions.erase(action);
    }
  }
}

} // end namespace action
} // end namespace eduart
} // end namespace robot
