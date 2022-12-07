/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/eduard/eduard.hpp>

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetGatewayBot : public eduart::robot::eduard::Eduard
{
public:
  EthernetGatewayBot();
  ~EthernetGatewayBot() override;

private:
  std::shared_ptr<rclcpp::TimerBase> _timer_process_status_report;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
