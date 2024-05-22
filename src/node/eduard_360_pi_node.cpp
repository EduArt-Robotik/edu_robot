/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"

#include <edu_robot/bot/eduard.hpp>

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>
#include <edu_robot/hardware/can_gateway/hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <cstddef>

using eduart::robot::bot::Eduard;
using eduart::robot::hardware::can_gateway::CanGatewayShield;
using eduart::robot::hardware::can_gateway::HardwareComponentFactory;
using eduart::robot::hardware::can_gateway::MotorControllerHardware;

class CanGatewayBot : public Eduard
{
public:
  CanGatewayBot()
    : Eduard(
        "eduard",
        std::make_unique<CanGatewayShield>(
          "eduart-can2", "eduart-can1", "eduart-can0"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

    // Lightings
    factory.addLighting("head")
           .addLighting("right_side")
           .addLighting("left_side")
           .addLighting("back")
           .addLighting("all");

    // Motor Controller
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string motor_controller_name = "motor_controller_" + std::to_string(i);
      const auto hardware_parameter = MotorControllerHardware::get_parameter(
        motor_controller_name, {}, *this);
      factory.addMotorController(motor_controller_name, hardware_parameter);
    }

    // Range Sensor
    factory.addRangeSensor("range/front/left", 0u)
           .addRangeSensor("range/front/right", 1u)
           .addRangeSensor("range/rear/left", 2u)
           .addRangeSensor("range/rear/right", 3u); 

    // IMU Sensor
    factory.addImuSensor("imu", 0x381);

    // Initialize
    initialize(factory);
    shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanGatewayBot>());
  rclcpp::shutdown();

  return 0;
}
