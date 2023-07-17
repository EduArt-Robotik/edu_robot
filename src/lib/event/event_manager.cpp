#include "edu_robot/event/event_manager.hpp"

namespace eduart {
namespace robot {
namespace event {

void EventManager::addEvent(std::shared_ptr<Event> event)
{
  _events.push_back(event);
}

void EventManager::process()
{
  for (auto event = _events.begin(); event != _events.end(); ++event) {
    if ((*event)->isReady()) {
      (*event)->process();
      _events.erase(event);
    }
  }
}

} // end namespace event
} // end namespace eduart
} // end namespace robot
