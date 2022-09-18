#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/color.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/lighting.hpp"
#include "edu_robot/iot_shield/motor_controller.hpp"
#include "edu_robot/motor_controller.hpp"
#include "edu_robot/robot.hpp"

#include <memory>

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
  auto iot_shield = std::dynamic_pointer_cast<IotShield>(_hardware_interface);

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


  // Motor controllers.
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
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
