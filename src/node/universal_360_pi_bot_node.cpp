/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/bot/universal_bot.hpp>

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>
#include <edu_robot/hardware/can_gateway/hardware_component_factory.hpp>
#include <edu_robot/hardware/can_gateway/motor_controller_hardware.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::bot::UniversalBot;
using eduart::robot::hardware::can_gateway::CanGatewayShield;
using eduart::robot::hardware::can_gateway::HardwareComponentFactory;
using eduart::robot::hardware::can_gateway::SensorTofRingHardware;
using eduart::robot::hardware::can_gateway::MotorControllerHardware;

class CanGatewayUniversalBot : public UniversalBot
{
public:
  CanGatewayUniversalBot()
    : UniversalBot(
        "universal_bot",
        std::make_unique<CanGatewayShield>(
          "eduart-can2", "eduart-can1", "eduart-can0"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

    // Motor Controller
    for (std::size_t i = 0; i < _parameter.axis.size(); ++i) {
      const std::string motor_controller_name = std::string("motor_controller_") + std::to_string(i);
      const auto hardware_parameter = MotorControllerHardware::get_parameter(
        motor_controller_name, {}, *this);
      factory.addMotorController(motor_controller_name, hardware_parameter);
    }

    // IMU Sensor
    factory.addImuSensor("imu", 0x381);

    // Pointcloud Sensor
    // auto point_cloud_parameter = SensorTofHardware::get_parameter(
    //   "pointcloud_left", {}, *this
    // );
    // factory.addTofSensor("pointcloud_left", point_cloud_parameter, *this);
    auto point_cloud_parameter = SensorTofRingHardware::get_parameter(
      "pointcloud_left",
      {"pointcloud_left_a", "pointcloud_left_b"},
      *this
    );
    factory.addTofRingSensor("pointcloud_left", point_cloud_parameter, *this);

    // Initialize
    initialize(factory);
    shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanGatewayUniversalBot>());
  rclcpp::shutdown();

  return 0;
}
