#include "edu_robot/ethernet_gateway/ethernet_gateway_flex_bot.hpp"
#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"
#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include "edu_robot/flex_bot/flex_bot.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

EthernetGatewayFlexBot::EthernetGatewayFlexBot()
  : eduart::robot::flex_bot::FlexBot(
    "flex_bot",
    // std::make_unique<EthernetGatewayShield>("192.168.1.20", 2345)
    std::make_unique<EthernetGatewayShield>("192.168.1.20", 1234)
  )
{
  auto shield = std::dynamic_pointer_cast<EthernetGatewayShield>(_hardware_interface);
  auto factory = HardwareComponentFactory(shield);
  // _timer_process_status_report = create_wall_timer(100ms, [shield]{ shield->processStatusReport(); });

          // Lightings
  factory//.addLighting("head", "head_lighting")
         //.addLighting("right_side", "right_side_lighting")
         //.addLighting("left_side", "left_side_lighting")
         //.addLighting("back", "back_lighting")
         //.addLighting("all", "all_lighting")
         // Motor Controller
         .addMotorController("motor", "motor_hardware")
         // Range Sensor
         //.addRangeSensor("range/front/left", "range/front/left/hardware",
         //                0u, RangeSensorHardware::Parameter{ })
         //.addRangeSensor("range/front/right", "range/front/right/hardware",
         //                1u, RangeSensorHardware::Parameter{ })
         //.addRangeSensor("range/rear/left", "range/rear/left/hardware",
         //                2u, RangeSensorHardware::Parameter{ })
         //.addRangeSensor("range/rear/right", "range/rear/right/hardware",
         //                3u, RangeSensorHardware::Parameter{ })
         // IMU Sensor
         .addImuSensor("imu", "imu_hardware", *this);

  initialize(factory);
  shield->registerComponentInput(_detect_charging_component);
}

EthernetGatewayFlexBot::~EthernetGatewayFlexBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
