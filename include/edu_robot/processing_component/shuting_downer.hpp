/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/processing_component/processing_component.hpp"

#include <edu_robot/event.hpp>

#include <functional>

namespace eduart {
namespace robot {
namespace processing {

class ShutingDowner : public ProcessingComponent
{
public:
  ShutingDowner(rclcpp::Node& ros_node, const std::function<void()> indicate_shutdown_callback = nullptr);

  void process() override;

private:
  std::function<void()> _indicate_shutdown_callback;
};

} // end namespace processing
} // end namespace eduart
} // end namespace robot
