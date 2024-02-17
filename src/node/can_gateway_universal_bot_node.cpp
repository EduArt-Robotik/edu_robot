/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/bot/universal_bot/universal_bot.hpp>

#include <edu_robot/hardware/can/can_gateway_shield.hpp>
#include <edu_robot/hardware/can/hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::hardware::can::CanGatewayShield;
using eduart::robot::hardware::can::HardwareComponentFactory;
using eduart::robot::hardware::can::SensorPointCloudHardware;

class CanGatewayUniversalBot : public eduart::robot::universal_bot::UniversalBot
{
public:
  CanGatewayUniversalBot()
    : eduart::robot::universal_bot::UniversalBot(
        "universal_bot",
        std::make_unique<CanGatewayShield>(
          "eduart-can0", "eduart-can1", "eduart-can2"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

    // Motor Controller
    for (std::size_t i = 0; i < _parameter.axis.size(); ++i) {
      factory.addMotorController(
        std::string("motor_controller_") + std::to_string(i),
        i | 0x400,
        i | 0x480
      );
    }

    // IMU Sensor
    factory.addImuSensor("imu", 0x381, *this);

    // Pointcloud Sensor
    auto point_cloud_parameter = SensorPointCloudHardware::get_parameter(
      "pointcloud_left", {}, *this
    );
    factory.addPointCloudSensor("pointcloud_left", point_cloud_parameter, *this);

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