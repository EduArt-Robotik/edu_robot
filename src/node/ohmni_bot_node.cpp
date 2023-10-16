/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include <edu_robot/ohmni_bot/ohmni_bot.hpp>
#include <edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp>

#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

class EthernetGatewayOhmniBot : public eduart::robot::ohmni_bot::OhmniBot
{
public:
  EthernetGatewayOhmniBot()
    : eduart::robot::ohmni_bot::OhmniBot(
        "ohmni_bot",
        std::make_unique<eduart::robot::ethernet::EthernetGatewayShield>(
          "192.168.2.20", 1234
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<eduart::robot::ethernet::EthernetGatewayShield>(_hardware_interface);
    auto factory = eduart::robot::ethernet::HardwareComponentFactory(shield);

    factory.addSingleChannelMotorController("motor", "motor_hardware")
           .addImuSensor("imu", "imu_hardware", *this);

    initialize(factory);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EthernetGatewayOhmniBot>());
  rclcpp::shutdown();

  return 0;
}
