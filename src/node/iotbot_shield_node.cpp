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
    factory.addLighting("head")
           .addLighting("right_side")
           .addLighting("left_side")
           .addLighting("back")
           .addLighting("all")
           // Motor Controller
           .addMotorController("motor_controller")
           // Range Sensor
           .addRangeSensor("range/front/left", 0u)
           .addRangeSensor("range/front/right", 1u)
           .addRangeSensor("range/rear/left", 2u)
           .addRangeSensor("range/rear/right", 3u)
           // IMU Sensor
           .addImuSensor("imu");

    initialize(factory);

    // Configure IoT Shield.
    // \todo move it into shield somehow.
    const bool imu_data_mode = std::dynamic_pointer_cast<eduart::robot::SensorImu>(_sensors.at("imu"))->parameter().raw_data_mode;
    iot_shield->registerComponentInput(_detect_charging_component);
    iot_shield->setImuRawDataMode(imu_data_mode);
    
    // Start up with robot mode INACTIVE.
    _mode_state_machine.switchToMode(eduart::robot::RobotMode::INACTIVE);

    // HACK! Timer should be in class IoTShield.
    _timer_process_status_report = create_wall_timer(
      20ms, std::bind(&IotShield::processStatusReport, iot_shield.get())
    );
  }

private:
  std::shared_ptr<rclcpp::TimerBase> _timer_process_status_report;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IotBot>());
  rclcpp::shutdown();

  return 0;
}
