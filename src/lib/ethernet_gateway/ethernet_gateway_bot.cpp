#include "edu_robot/ethernet_gateway/ethernet_gateway_bot.hpp"
#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"
#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"

#include <algorithm>
#include <memory>

#include <rclcpp/create_timer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

EthernetGatewayBot::EthernetGatewayBot()
  : eduart::robot::eduard::Eduard(
    "eduard",
    // std::make_unique<EthernetGatewayShield>("192.168.1.20", 2345)
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
}

EthernetGatewayBot::~EthernetGatewayBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
