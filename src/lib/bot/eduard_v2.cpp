#include "edu_robot/bot/eduard_v2.hpp"

namespace eduart {
namespace robot {
namespace bot {

EduardV2::EduardV2(
  const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns)
  : Eduard(robot_name, std::move(hardware_interface), ns)
{

}

EduardV2::~EduardV2()
{

}

void EduardV2::initialize(eduart::robot::HardwareComponentFactory& factory)
{
  // First Calling initialize of Base Class
  Eduard::initialize(factory);

  // Lightings
  registerLighting(std::make_shared<robot::Lighting>(
    "head",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.hardware().at("head")->cast<robot::Lighting::ComponentInterface>()
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "right_side",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.hardware().at("right_side")->cast<robot::Lighting::ComponentInterface>()
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "left_side",
    COLOR::DEFAULT::BACK,
    1.0f,
    factory.hardware().at("left_side")->cast<robot::Lighting::ComponentInterface>()
  ));
  registerLighting(std::make_shared<robot::Lighting>(
    "back",
    COLOR::DEFAULT::BACK,
    1.0f,
    factory.hardware().at("back")->cast<robot::Lighting::ComponentInterface>()
  ));

  // Use all representation to set a initial light.
  auto lighting_all = std::make_shared<robot::Lighting>(
    "all",
    COLOR::DEFAULT::HEAD,
    1.0f,
    factory.hardware().at("all")->cast<robot::Lighting::ComponentInterface>()
  );
  lighting_all->setColor(COLOR::DEFAULT::BACK, Lighting::Mode::RUNNING);
  registerLighting(lighting_all);

  // Range Sensors
  constexpr std::array<const char*, 4> range_sensor_name = {
    "range/front/left", "range/front/right", "range/rear/left", "range/rear/right" };
  const std::array<tf2::Transform, 4> range_sensor_pose = {
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17,  0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3( 0.17, -0.063, 0.045)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17,  0.063, 0.050)),
    tf2::Transform(tf2::Quaternion(0.0, 0.0, 1.0, 0.0), tf2::Vector3(-0.17, -0.063, 0.050))
  };
  constexpr eduart::robot::SensorRange::Parameter range_sensor_parameter{ 10.0 * M_PI / 180.0, 0.01, 5.0 };

  for (std::size_t i = 0; i < range_sensor_name.size(); ++i) {
    auto range_sensor = std::make_shared<robot::SensorRange>(
      range_sensor_name[i],
      getFrameIdPrefix() + range_sensor_name[i],
      getFrameIdPrefix() + Robot::_parameter.tf.base_frame,
      range_sensor_pose[i],
      range_sensor_parameter,
      *this,
      factory.hardware().at(range_sensor_name[i])->cast<robot::SensorRange::SensorInterface>()
    );
    registerSensor(range_sensor);
    range_sensor->registerComponentInput(_collision_avoidance_component);
    factory.hardware().at(range_sensor_name[i])->cast<robot::SensorRange::SensorInterface>()->initialize(range_sensor_parameter);
  }
}

} // end namespace bot
} // end namespace robot
} // end namespace eduart
