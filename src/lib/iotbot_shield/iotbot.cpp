#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/eduard/eduard.hpp"
#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/lighting_hardware.hpp"
#include "edu_robot/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/iot_shield/range_sensor_hardware.hpp"
#include "edu_robot/iot_shield/iotbot_hardware_component_factory.hpp"

#include <algorithm>
#include <memory>

#include <rclcpp/create_timer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

IotBot::IotBot()
  : eduart::robot::eduard::Eduard(
      "eduard",
      std::make_unique<IotShield>("/dev/ttyS1")
    )
{
  auto iot_shield = std::dynamic_pointer_cast<IotShield>(_hardware_interface);
  auto factory = IotBotHardwareComponentFactory(iot_shield);
  _timer_process_status_report = create_wall_timer(100ms, [iot_shield]{ iot_shield->processStatusReport(); });

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
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
