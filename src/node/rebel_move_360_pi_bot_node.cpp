/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */

#include <edu_robot/bot/rebel_move.hpp>

#include <edu_robot/hardware/igus/hardware_component_factory.hpp>
#include <edu_robot/hardware/igus/can_gateway_shield.hpp>
#include <edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp>
#include <edu_robot/hardware/igus/motor_controller_hardware.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>

using eduart::robot::bot::RebelMove;
using eduart::robot::hardware::igus::CanGatewayShield;
using eduart::robot::hardware::igus::HardwareComponentFactory;
using eduart::robot::hardware::igus::MotorControllerHardware;
using eduart::robot::hardware::can_gateway::SensorTofRingHardware;

class RebelMove360Bot : public RebelMove
{
public:
  RebelMove360Bot()
    : RebelMove(
        "turtle", std::make_unique<CanGatewayShield>(
          "eduart-can2", "eduart-can1", "eduart-can0"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);

    // Motor Controller
    std::array<char const *const, 4> motor_controller_name = {
      "motor_controller_a", "motor_controller_a", "motor_controller_a", "motor_controller_a"};

    for (const auto& name : motor_controller_name) {
      const auto parameter = MotorControllerHardware::get_parameter(
        name, {}, *this);
      factory.addMotorController(name, parameter);
    }

    // ToF Sensor Ring
    auto point_cloud_parameter = SensorTofRingHardware::get_parameter(
      "pointcloud_left",
      {"pointcloud_left_a", "pointcloud_left_b"},
      *this
    );
    factory.addTofRingSensor("pointcloud_left", point_cloud_parameter, *this);
    
    // IMU Sensor
    factory.addImuSensor("imu", 0x381);           

    initialize(factory);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RebelMove360Bot>());
  rclcpp::shutdown();

  return 0;
}
