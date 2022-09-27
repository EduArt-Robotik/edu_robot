/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/iot_shield/iotbot.hpp>

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto iotbot = std::make_shared<eduart::robot::iotbot::IotBot>();
  iotbot->initialize();
  rclcpp::spin(iotbot);
  rclcpp::shutdown();

  return 0;
}