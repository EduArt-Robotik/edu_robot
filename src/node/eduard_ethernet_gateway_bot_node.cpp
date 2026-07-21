/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/bot/eduard_v2.hpp"

#include <edu_robot/hardware/ethernet_gateway/hardware_component_factory.hpp>
#include <edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp>
#include <edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::bot::EduardV2;
using eduart::robot::hardware::ethernet::EthernetGatewayShield;
using eduart::robot::hardware::ethernet::HardwareComponentFactory;
using eduart::robot::hardware::ethernet::MotorControllerHardware;

class EduardEthernetGatewayBot : public EduardV2
{
public:
  EduardEthernetGatewayBot()
    : EduardV2(
        "eduard",
        std::make_unique<EthernetGatewayShield>("192.168.2.20", 1234)
      )
  {
    auto shield = std::dynamic_pointer_cast<EthernetGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);

    // Lightings
    factory.addLighting("head")
           .addLighting("right_side")
           .addLighting("left_side")
           .addLighting("back")
           .addLighting("all");

    // Motor Controller
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string motor_controller_name = "motor_controller_" + std::to_string(i);
      const auto hardware_parameter = MotorControllerHardware<2>::get_parameter(
        motor_controller_name, {}, *this);
      factory.addMotorController(motor_controller_name, hardware_parameter);
    }

    // Range Sensor
    factory.addRangeSensor("range/front/left", 0u)
           .addRangeSensor("range/front/right", 1u)
           .addRangeSensor("range/rear/left", 2u)
           .addRangeSensor("range/rear/right", 3u);

    // IMU Sensor
    factory.addImuSensor("imu");

    initialize(factory);
    shield->output("system.voltage")->connect(_detect_charging_component->input("voltage"));
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EduardEthernetGatewayBot>());
  rclcpp::shutdown();

  return 0;
}
