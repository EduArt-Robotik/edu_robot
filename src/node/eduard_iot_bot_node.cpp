/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include "edu_robot/bot/eduard.hpp"
#include "edu_robot/hardware/iot_shield/motor_controller_hardware.hpp"

#include <edu_robot/hardware/iot_shield/iot_shield.hpp>
#include <edu_robot/hardware/iot_shield/iotbot_hardware_component_factory.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::robot::bot::Eduard;
using eduart::robot::hardware::iot_shield::IotShield;
using eduart::robot::hardware::iot_shield::IotBotHardwareComponentFactory;
using eduart::robot::hardware::iot_shield::MotorControllerHardware;

using namespace std::chrono_literals;

class EduardIotBot : public Eduard
{
public:
  EduardIotBot()
    : Eduard(
        "eduard",
        std::make_unique<IotShield>("/dev/ttyS1")
    )
  {
    auto iot_shield = std::dynamic_pointer_cast<IotShield>(_hardware_interface);
    auto factory = IotBotHardwareComponentFactory(iot_shield);

    // Lightings
    factory.addLighting("head")
           .addLighting("right_side")
           .addLighting("left_side")
           .addLighting("back")
           .addLighting("all");

    // Motor Controller
    const auto motor_controller_parameter = MotorControllerHardware::get_parameter(
      "motor_controller", {}, *this);
    factory.addMotorController("motor_controller", motor_controller_parameter);

    // Range Sensor
    factory.addRangeSensor("range/front/left", 0u)
           .addRangeSensor("range/front/right", 1u)
           .addRangeSensor("range/rear/left", 2u)
           .addRangeSensor("range/rear/right", 3u);

    // IMU Sensor
    factory.addImuSensor("imu");

    initialize(factory);

    // Configure IoT Shield.
    // \todo move it into shield somehow.
    const bool imu_data_mode = std::dynamic_pointer_cast<eduart::robot::SensorImu>(_sensors.at("imu"))->parameter().raw_data_mode;
    iot_shield->registerComponentInput(_detect_charging_component);
    iot_shield->setImuRawDataMode(imu_data_mode);
    
    // Start up with robot mode INACTIVE.
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);
  }

private:
  std::shared_ptr<rclcpp::TimerBase> _timer_process_status_report;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EduardIotBot>());
  rclcpp::shutdown();

  return 0;
}
