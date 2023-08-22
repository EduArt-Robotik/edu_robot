/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <edu_robot/eduard/eduard.hpp>

#include <edu_robot/iot_shield/iot_shield.hpp>
#include <edu_robot/iot_shield/iotbot_hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::iotbot::IotShield;
using eduart::robot::iotbot::IotBotHardwareComponentFactory;

using namespace std::chrono_literals;

class IotBot : public eduart::robot::eduard::Eduard
{
public:
  IotBot()
    : eduart::robot::eduard::Eduard(
        "eduard",
        std::make_unique<IotShield>("/dev/ttyS1")
    )
  {
    auto iot_shield = std::dynamic_pointer_cast<IotShield>(_hardware_interface);
    auto factory = IotBotHardwareComponentFactory(iot_shield);

            // Lightings
    factory.addLighting("head", "head_lighting")
           .addLighting("right_side", "right_side_lighting")
           .addLighting("left_side", "left_side_lighting")
           .addLighting("back", "back_lighting")
           .addLighting("all", "all_lighting")
           // Motor Controller
           .addMotorController("motor", "motor_hardware")
           // Range Sensor
           .addRangeSensor("range/front/left", "range/front/left/hardware", 0u)
           .addRangeSensor("range/front/right", "range/front/right/hardware", 1u)
           .addRangeSensor("range/rear/left", "range/rear/left/hardware", 2u)
           .addRangeSensor("range/rear/right", "range/rear/right/hardware", 3u)
           // IMU Sensor
           .addImuSensor("imu", "imu_hardware");

    initialize(factory);
    iot_shield->registerComponentInput(_detect_charging_component);
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);    
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IotBot>());
  rclcpp::shutdown();

  return 0;
}
