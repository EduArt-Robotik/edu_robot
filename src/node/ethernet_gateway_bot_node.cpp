/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/eduard/eduard.hpp>

#include <edu_robot/ethernet_gateway/hardware_component_factory.hpp>
#include <edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::ethernet::EthernetGatewayShield;
using eduart::robot::ethernet::HardwareComponentFactory;

class EthernetGatewayBot : public eduart::robot::eduard::Eduard
{
public:
  EthernetGatewayBot() 
    : eduart::robot::eduard::Eduard(
        "eduard",
        std::make_unique<EthernetGatewayShield>("192.168.2.20", 1234)
      )
  {
    auto shield = std::dynamic_pointer_cast<EthernetGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);
    // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

            // Lightings
    factory.addLighting("head", "head_lighting")
           .addLighting("right_side", "right_side_lighting")
           .addLighting("left_side", "left_side_lighting")
           .addLighting("back", "back_lighting")
           .addLighting("all", "all_lighting")
           // Motor Controller
           .addMotorController("motor", "motor_hardware")
           // Range Sensor
           .addRangeSensor("range/front/left", "range/front/left/hardware", 0u, *this)
           .addRangeSensor("range/front/right", "range/front/right/hardware", 1u, *this)
           .addRangeSensor("range/rear/left", "range/rear/left/hardware", 2u, *this)
           .addRangeSensor("range/rear/right", "range/rear/right/hardware", 3u, *this)
           // IMU Sensor
           .addImuSensor("imu", "imu_hardware", *this);

    initialize(factory);
    shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EthernetGatewayBot>());
  rclcpp::shutdown();

  return 0;
}
