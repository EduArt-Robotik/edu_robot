/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/eduard/eduard.hpp>

#include <edu_robot/hardware/ethernet_gateway/hardware_component_factory.hpp>
#include <edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp>

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
    factory.addLighting("head")
           .addLighting("right_side")
           .addLighting("left_side")
           .addLighting("back")
           .addLighting("all")
           // Motor Controller
           .addMotorController("motor_controller_0", 0)
           .addMotorController("motor_controller_1", 1)
           // Range Sensor
           .addRangeSensor("range/front/left", 0u, *this)
           .addRangeSensor("range/front/right", 1u, *this)
           .addRangeSensor("range/rear/left", 2u, *this)
           .addRangeSensor("range/rear/right", 3u, *this)
           // IMU Sensor
           .addImuSensor("imu", *this);

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
