#include "edu_robot/processing_component/shuting_downer.hpp"

namespace eduart {
namespace robot {
namespace processing {

ShutingDowner::ShutingDowner(rclcpp::Node& ros_node)
  : ProcessingComponent("shuting_downer", ros_node)
{
  createInput<Event>("event");
}

void ShutingDowner::process()
{
  for (auto port = input("event"); port->hasValue();) {
    // process event
    if (port->getValue().get<Event>() == Event::SHUTDOWN) {
      RCLCPP_INFO(rclcpp::get_logger("ShutingDowner"), "Shuting down the system now.");
      if (system("sudo systemctl poweroff")) { // its okay to use unsafe function here. Requires sudo rights anyway.
        RCLCPP_ERROR(rclcpp::get_logger("ShutingDowner"), "Failed to shutdown the system!");
      }
    }
  }
}

} // end namespace processing
} // end namespace eduart
} // end namespace robot
