#include "edu_robot/hardware/iot_shield/iotbot_hardware_component_factory.hpp"

#include "edu_robot/hardware/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/iot_shield/lighting_hardware.hpp"
#include "edu_robot/hardware/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/hardware/iot_shield/range_sensor_hardware.hpp"

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

IotBotHardwareComponentFactory& IotBotHardwareComponentFactory::addLighting(const std::string& lighting_name)
{
  _hardware[lighting_name] = std::make_unique<LightingHardware>(
    lighting_name, _shield->getCommunicator()
  );

  return *this;
} 

IotBotHardwareComponentFactory& IotBotHardwareComponentFactory::addMotorController(
  const std::string& controller_name, const MotorControllerHardware::Parameter& parameter)
{
  auto compound_motor = std::make_shared<MotorControllerHardware>(
    controller_name, parameter, _shield->getCommunicator());

  _motor_controller_hardware.push_back(compound_motor);

  return *this;
}

IotBotHardwareComponentFactory& 
IotBotHardwareComponentFactory::addRangeSensor(const std::string& sensor_name, const std::uint8_t id)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(id, _shield->getCommunicator());
  _hardware[sensor_name] = range_sensor_hardware;

  return *this;
}                                               

IotBotHardwareComponentFactory& IotBotHardwareComponentFactory::addImuSensor(const std::string& sensor_name)
{
  auto imu_hardware = std::make_shared<ImuSensorHardware>(_shield->getCommunicator());
  _hardware[sensor_name] = imu_hardware;

  return *this;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
