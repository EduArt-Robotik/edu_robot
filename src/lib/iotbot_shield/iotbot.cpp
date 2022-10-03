#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/eduard/eduard.hpp"
#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/lighting_hardware.hpp"
#include "edu_robot/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/iot_shield/range_sensor_hardware.hpp"

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
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
