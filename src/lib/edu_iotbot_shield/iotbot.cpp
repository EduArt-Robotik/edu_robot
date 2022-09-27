#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/color.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/lighting.hpp"
#include "edu_robot/iot_shield/motor_controller.hpp"
#include "edu_robot/iot_shield/range_sensor.hpp"
#include "edu_robot/motor_controller.hpp"
#include "edu_robot/range_sensor.hpp"
#include "edu_robot/robot.hpp"

#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace eduart {
namespace robot {
namespace iotbot {

struct COLOR {
  struct DEFAULT {
    static constexpr Color HEAD = { 0xff, 0xff, 0xff };
    static constexpr Color BACK = { 0xff, 0x00, 0x00 };
  };
};
IotBot::IotBot()
  : eduart::robot::Robot("IotBot", std::make_unique<IotShield>("/dev/ttyS1"))
{

}

void IotBot::initialize()
{
  auto iot_shield = std::dynamic_pointer_cast<IotShield>(_hardware_interface);

  // Lightings
  registerLighting(std::make_shared<iotbot::Lighting>(
    "head",
    0u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::HEAD,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "right_side",
    1u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::HEAD,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "left_side",
    2u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::BACK,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "back",
    3u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::BACK,
    1.0f)
  );

  // Use all representation to set a initial light.
  auto lighting_all = std::make_shared<iotbot::Lighting>(
    "all",
    4u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::HEAD,
    1.0f
  );
  lighting_all->setColor(COLOR::DEFAULT::BACK, Lighting::Mode::RUNNING);
  registerLighting(lighting_all);


  // Motor Controllers
  robot::MotorController::Parameter parameter;
  auto compound_motor_controller = std::make_shared<iotbot::CompoundMotorController>(
    "motor_d",
    iot_shield->getCommunicator(),
    parameter
  );

  registerMotorController(compound_motor_controller->dummyMotorController()[0]);
  registerMotorController(compound_motor_controller->dummyMotorController()[1]);
  registerMotorController(compound_motor_controller->dummyMotorController()[2]);
  registerMotorController(compound_motor_controller);

  // Range Sensors
  constexpr eduart::robot::RangeSensor::Parameter range_sensor_parameter{ 10.0 * M_PI / 180.0, 0.01, 5.0 };

  auto range_sensor = std::make_shared<iotbot::RangeSensor>(
    "front_left",
    "range/front/left",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.17, 0.07, 0.1)),
    0u,
    range_sensor_parameter,
    shared_from_this()
  );
  registerSensor(range_sensor);
  iot_shield->registerIotShieldRxDevice(range_sensor);

  range_sensor = std::make_shared<iotbot::RangeSensor>(
    "front_right",
    "range/front/right",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(0.17, -0.07, 0.1)),
    1u,
    range_sensor_parameter,
    shared_from_this()
  );
  registerSensor(range_sensor);
  iot_shield->registerIotShieldRxDevice(range_sensor);

  range_sensor = std::make_shared<iotbot::RangeSensor>(
    "rear_left",
    "range/rear/left",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, 0.07, 0.1)),
    2u,
    range_sensor_parameter,
    shared_from_this()
  );
  registerSensor(range_sensor);
  iot_shield->registerIotShieldRxDevice(range_sensor);

  range_sensor = std::make_shared<iotbot::RangeSensor>(
    "rear_right",
    "range/rear/right",
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, -0.07, 0.1)),
    3u,
    range_sensor_parameter,
    shared_from_this()
  );
  registerSensor(range_sensor);
  iot_shield->registerIotShieldRxDevice(range_sensor);
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
