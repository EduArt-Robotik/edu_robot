/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/bot/eduard.hpp>

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>
#include <edu_robot/hardware/can_gateway/hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::bot::Eduard;
using eduart::robot::hardware::can_gateway::CanGatewayShield;
using eduart::robot::hardware::can_gateway::HardwareComponentFactory;

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
           .addLighting("all")
           // Motor Controller
           .addMotorController("motor_controller_0", 0 | 0x400, 0 | 0x480)
           .addMotorController("motor_controller_1", 1 | 0x400, 1 | 0x480)
           // Range Sensor
           .addRangeSensor("range/front/left", 0u)
           .addRangeSensor("range/front/right", 1u)
           .addRangeSensor("range/rear/left", 2u)
           .addRangeSensor("range/rear/right", 3u)
           // IMU Sensor
           .addImuSensor("imu", 0x381);

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
