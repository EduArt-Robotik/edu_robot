/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include <edu_robot/ohmni_bot/ohmni_bot.hpp>
#include <edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto hardware_interface = std::make_unique<eduart::robot::ethernet::EthernetGatewayShield>("192.168.2.20", 1234);
  auto ohmni_bot = std::make_shared<eduart::robot::ohmni_bot::OhmniBot>("ohmni_bot", std::move(hardware_interface));
  auto shield = std::dynamic_pointer_cast<eduart::robot::ethernet::EthernetGatewayShield>(ohmni_bot->getHardwareInterface());
  auto factory = eduart::robot::ethernet::HardwareComponentFactory(shield);

  factory.addSingleChannelMotorController("motor", "motor_hardware")
         .addImuSensor("imu", "imu_hardware", *ohmni_bot);

  ohmni_bot->initialize(factory);

  rclcpp::spin(ohmni_bot);
  rclcpp::shutdown();

  return 0;
}
