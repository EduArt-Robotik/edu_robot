/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"

#include <edu_robot/bot/eduard_v3.hpp>

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>
#include <edu_robot/hardware/can_gateway/hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <cstddef>

using eduart::robot::bot::EduardV3;
using eduart::robot::hardware::can_gateway::CanGatewayShield;
using eduart::robot::hardware::can_gateway::HardwareComponentFactory;
using eduart::robot::hardware::can_gateway::MotorControllerHardware;

class Eduard360PiBot : public EduardV3
{
public:
  Eduard360PiBot()
    : EduardV3(
        "eduard",
        std::make_unique<CanGatewayShield>(
          "eduart-can2", "eduart-can1", "eduart-can0"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

    // Motor Controller
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string motor_controller_name = "motor_controller_" + std::to_string(i);
      const auto hardware_parameter = MotorControllerHardware::get_parameter(
        motor_controller_name, {}, *this);
      factory.addMotorController(motor_controller_name, hardware_parameter);
    }

    // ToF Sensor Ring
    constexpr std::size_t tof_num_sensors = 14;
    std::vector<std::string> tof_sensor_ring_a_names;
    std::vector<std::string> tof_sensor_ring_b_names;

    for (std::size_t i = 1; i <= tof_num_sensors / 2; ++i) {
      tof_sensor_ring_a_names.emplace_back(std::string("tof_pointcloud_sensor_") + std::to_string(i));
    }
    for (std::size_t i = tof_num_sensors / 2 + 1; i <= tof_num_sensors; ++i) {
      tof_sensor_ring_b_names.emplace_back(std::string("tof_pointcloud_sensor_") + std::to_string(i));
    }

    factory.addTofRingSensor(
      "tof_sensor_ring", tof_sensor_ring_a_names, tof_sensor_ring_b_names, *this);

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
  rclcpp::spin(std::make_shared<Eduard360PiBot>());
  rclcpp::shutdown();

  return 0;
}
