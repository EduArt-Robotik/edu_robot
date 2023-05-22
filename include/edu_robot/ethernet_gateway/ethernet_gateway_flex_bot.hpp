/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/flex_bot/flex_bot.hpp>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetGatewayFlexBot : public eduart::robot::flex_bot::FlexBot
{
public:
  EthernetGatewayFlexBot();
  ~EthernetGatewayFlexBot() override;

private:
  std::shared_ptr<rclcpp::TimerBase> _timer_process_status_report;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
