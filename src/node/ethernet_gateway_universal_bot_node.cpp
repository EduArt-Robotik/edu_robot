/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/bot/universal_bot/universal_bot.hpp>

#include <edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp>
#include <edu_robot/hardware/ethernet_gateway/hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::hardware::ethernet::EthernetGatewayShield;
using eduart::robot::hardware::ethernet::HardwareComponentFactory;

class EthernetGatewayUniversalBot : public eduart::robot::universal_bot::UniversalBot
{
public:
  EthernetGatewayUniversalBot()
    : eduart::robot::universal_bot::UniversalBot(
        "universal_bot",
        std::make_unique<EthernetGatewayShield>("192.168.2.20", 1234)
      )
  {
    auto shield = std::dynamic_pointer_cast<EthernetGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

    // Motor Controller
    for (std::size_t i = 0; i < _parameter.axis.size(); ++i) {
      factory.addMotorController(std::string("motor_controller_") + std::to_string(i), i);
    }

    // IMU Sensor
    factory.addImuSensor("imu", *this);

    initialize(factory);
    shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EthernetGatewayUniversalBot>());
  rclcpp::shutdown();

  return 0;
}
