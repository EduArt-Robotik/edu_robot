#include "edu_robot/iot_shield/iotbot_hardware_component_factory.hpp"

#include "edu_robot/iot_shield/imu_sensor_hardware.hpp"
#include "edu_robot/iot_shield/lighting_hardware.hpp"
#include "edu_robot/iot_shield/motor_controller_hardware.hpp"
#include "edu_robot/iot_shield/range_sensor_hardware.hpp"
#include <functional>
#include <memory>

namespace eduart {
namespace robot {
namespace iotbot {

IotBotHardwareComponentFactory&
IotBotHardwareComponentFactory::addLighting(const std::string& lighting_name, const std::string& hardware_name)
{
  _lighting_hardware[lighting_name] = std::make_unique<iotbot::LightingHardware>(hardware_name, _shield->getCommunicator());
  return *this;
} 

IotBotHardwareComponentFactory&
IotBotHardwareComponentFactory::addMotorController(const std::string& motor_name, const std::string& hardware_name)
{
  auto compound_motor = std::make_shared<CompoundMotorControllerHardware>(hardware_name, _shield->getCommunicator());

  _motor_controller_hardware[motor_name + "_a"] = compound_motor->dummyMotorController()[0];
  _motor_controller_hardware[motor_name + "_b"] = compound_motor->dummyMotorController()[1];
  _motor_controller_hardware[motor_name + "_c"] = compound_motor->dummyMotorController()[2];
  _motor_controller_hardware[motor_name + "_d"] = compound_motor;

  _motor_sensor_hardware[motor_name + "_a"] = compound_motor->dummyMotorController()[0];
  _motor_sensor_hardware[motor_name + "_b"] = compound_motor->dummyMotorController()[1];
  _motor_sensor_hardware[motor_name + "_c"] = compound_motor->dummyMotorController()[2];
  _motor_sensor_hardware[motor_name + "_d"] = compound_motor;

  _shield->registerIotShieldRxDevice(compound_motor);

  return *this;
}

IotBotHardwareComponentFactory&
IotBotHardwareComponentFactory::addRangeSensor(
  const std::string& sensor_name, const std::string& hardware_name, const std::uint8_t id)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(hardware_name, id);
  _shield->registerIotShieldRxDevice(range_sensor_hardware);
  _range_sensor_hardware[sensor_name] = range_sensor_hardware;
  return *this;
}                                               
  
IotBotHardwareComponentFactory&
IotBotHardwareComponentFactory::addImuSensor(const std::string& sensor_name, const std::string& hardware_name)
{
  auto imu_hardware = std::make_shared<ImuSensorHardware>(hardware_name, _shield->getCommunicator());
  _shield->registerIotShieldRxDevice(imu_hardware);
  _imu_sensor_hardware[sensor_name] = imu_hardware;
  return *this;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
