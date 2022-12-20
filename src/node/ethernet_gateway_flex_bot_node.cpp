/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/ethernet_gateway/ethernet_gateway_flex_bot.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::robot::ethernet::EthernetGatewayFlexBot>());
  rclcpp::shutdown();

  return 0;
}
