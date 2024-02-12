/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/bot/turtle/turtle.hpp>

#include <edu_robot/hardware/igus/hardware_component_factory.hpp>
#include <edu_robot/hardware/igus/can_gateway_shield.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::hardware::igus::CanGatewayShield;
using eduart::robot::hardware::igus::HardwareComponentFactory;

class IgusCanTurtleBot : public eduart::robot::turtle::Turtle
{
public:
  IgusCanTurtleBot()
    : eduart::robot::turtle::Turtle(
        "turtle", std::make_unique<CanGatewayShield>(
          "can0", "can1", "can2"
        )
      )
  {
    auto shield = std::dynamic_pointer_cast<CanGatewayShield>(_hardware_interface);
    auto factory = HardwareComponentFactory(shield);

    factory.addMotorController("motor_controller_a", 0x10)
          //  .addMotorController("motor_controller_b", 0x30)
          //  .addMotorController("motor_controller_c", 0x40)
          //  .addMotorController("motor_controller_d", 0x50)
           .addPointCloudSensor("pointcloud_left", {}, *this);

    initialize(factory);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IgusCanTurtleBot>());
  rclcpp::shutdown();

  return 0;
}
