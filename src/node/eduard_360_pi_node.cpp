/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/processing_component/shuting_downer.hpp"

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
    // creating HAL objects
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
    
    // Lighting
    factory.addLighting();

    // ToF Sensor Ring
    std::vector<std::string> tof_sensors_left  = {"front", "rear"};
    std::vector<std::string> tof_sensors_right = {"front", "rear"};

    factory.addTofRingSensor(
      "tof_sensor_ring", tof_sensors_left, tof_sensors_right, *this);

    // IMU Sensor
    factory.addImuSensor("imu", 0x381);

    // Initialize using created factory
    initialize(factory);

    // Connect Hardware Related Components
    shield->output("system.voltage")->connect(_detect_charging_component->input("voltage"));
    
    // \todo: shutting downer component should be moved to Eduard base class
    auto shutting_downer = std::make_shared<eduart::robot::processing::ShutingDowner>(*this , [this](){
      _mode_state_machine.switchToMode(eduart::robot::RobotMode::SHUTTING_DOWN);
    });
    shield->output("event")->connect(shutting_downer->input("event"));
    _processing_components.push_back(shutting_downer);

    // Switch to inactive mode (default)
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
