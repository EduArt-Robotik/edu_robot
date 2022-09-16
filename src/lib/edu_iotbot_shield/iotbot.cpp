#include "edu_robot/iot_shield/iotbot.hpp"
#include "edu_robot/color.hpp"
#include "edu_robot/iot_shield/iot_shield.hpp"
#include "edu_robot/iot_shield/lighting.hpp"

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
    "head_left",
    0u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::HEAD,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "head_right",
    1u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::HEAD,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "back_left",
    2u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::BACK,
    1.0f)
  );
  registerLighting(std::make_shared<iotbot::Lighting>(
    "back_right",
    3u,
    iot_shield->getCommunicator(),
    COLOR::DEFAULT::BACK,
    1.0f)
  );
}

IotBot::~IotBot()
{
  // \todo cleanup some hardware stuff here, related to iot shield only!
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
