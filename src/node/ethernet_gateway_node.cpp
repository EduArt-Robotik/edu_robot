/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto shield = std::make_shared<eduart::robot::ethernet::EthernetGatewayShield>("192.168.1.20", 2345);
  // rclcpp::spin();
  shield.reset();
  rclcpp::shutdown();

  return 0;
}
