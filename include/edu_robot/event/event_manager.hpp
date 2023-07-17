/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/event/event.hpp"

#include <memory>
#include <list>

namespace eduart {
namespace robot {
namespace event {

class EventManager
{
public:
  EventManager() = default;

  /**
   * \brief This method needs be called for managing all registered events.
   */
  void process();

  void addEvent(std::shared_ptr<Event> event);

private:
  std::list<std::shared_ptr<Event>> _events;
};

} // end namespace event
} // end namespace eduart
} // end namespace robot
