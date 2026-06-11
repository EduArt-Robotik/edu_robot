/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/bot/thn_georg.hpp"

#include <edu_robot/hardware/ethernet_gateway/hardware_component_factory.hpp>
#include <edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp>
#include <edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::bot::ThnGeorg;
using eduart::robot::hardware::ethernet::EthernetGatewayShield;
using eduart::robot::hardware::ethernet::HardwareComponentFactory;
using eduart::robot::hardware::ethernet::MotorControllerHardware;

class ThnGeorgBot : public ThnGeorg
{
public:
  ThnGeorgBot() 
    : ThnGeorg(
        "thn_georg",
        std::make_unique<EthernetGatewayShield>("192.168.2.20", 1234)
      )
  {
    auto shield = std::dynamic_pointer_cast<EthernetGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);

    // Motor Controller
    for (std::size_t i = 0; i < 4; ++i) {
      const std::string motor_controller_name = "motor_controller_" + std::to_string(i);
      const auto hardware_parameter = MotorControllerHardware<1>::get_parameter(
        motor_controller_name, {}, *this);
      factory.addSingleChannelMotorController(motor_controller_name, hardware_parameter);
    }

    // IMU Sensor
    factory.addImuSensor("imu");

    initialize(factory);
    shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThnGeorgBot>());
  rclcpp::shutdown();

  return 0;
}
