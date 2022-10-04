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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace eduart {
namespace robot {
namespace iotbot {

IotBot::IotBot()
  : eduart::robot::eduard::Eduard("IotBot", std::make_unique<IotShield>("/dev/ttyS1"))
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
         .addMotorController("motor", "motor_hardware", robot::MotorController::Parameter{ })
         // Range Sensor
         .addRangeSensor("range/front/left", "range/front/left/hardware",
                         0u, robot::RangeSensor::Parameter{ })
         .addRangeSensor("range/front/right", "range/front/right/hardware",
                         1u, robot::RangeSensor::Parameter{ })
         .addRangeSensor("range/rear/left", "range/rear/left/hardware",
                         2u, robot::RangeSensor::Parameter{ })
         .addRangeSensor("range/rear/right", "range/rear/right/hardware",
                         3u, robot::RangeSensor::Parameter{ })
         // IMU Sensor
         .addImuSensor("imu", "imu_hardware", robot::ImuSensor::Parameter{ });

  initialize(
    factory.lightingHardware(),
    factory.motorControllerHardware(),
    factory.motorSensorHardware(),
    factory.rangeSensorHardware(),
    factory.imuSensorHardware()
  );
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
