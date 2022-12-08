#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"

#include <functional>
#include <memory>

namespace eduart {
namespace robot {
namespace ethernet {

HardwareComponentFactory&
HardwareComponentFactory::addLighting(const std::string& lighting_name, const std::string& hardware_name)
{
  _lighting_hardware[lighting_name] = std::make_unique<ethernet::LightingHardware>(
    hardware_name, _shield->getCommunicator()
  );

  return *this;
} 

HardwareComponentFactory&
HardwareComponentFactory::addMotorController(
  const std::string& motor_name, const std::string& hardware_name,
  const eduart::robot::MotorController::Parameter& parameter)
{
  auto compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_a", hardware_name + "_b", parameter, 0, _shield->getCommunicator()
  );

  _motor_controller_hardware[motor_name + "_a"] = compound_motor->dummyMotorController();
  _motor_controller_hardware[motor_name + "_b"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_a"] = compound_motor->dummyMotorController();
  _motor_sensor_hardware[motor_name + "_b"] = compound_motor;

  compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_c", hardware_name + "_d", parameter, 1, _shield->getCommunicator()
  );  
  _motor_controller_hardware[motor_name + "_c"] = compound_motor->dummyMotorController();
  _motor_controller_hardware[motor_name + "_d"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_c"] = compound_motor->dummyMotorController();
  _motor_sensor_hardware[motor_name + "_d"] = compound_motor;

  compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_e", hardware_name + "_f", parameter, 2, _shield->getCommunicator()
  );  
  _motor_controller_hardware[motor_name + "_e"] = compound_motor->dummyMotorController();
  _motor_controller_hardware[motor_name + "_f"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_e"] = compound_motor->dummyMotorController();
  _motor_sensor_hardware[motor_name + "_f"] = compound_motor;

  // _shield->registerIotShieldRxDevice(compound_motor);

  return *this;
}

HardwareComponentFactory&
HardwareComponentFactory::addRangeSensor(const std::string& sensor_name, const std::string& hardware_name,
                                               const std::uint8_t id, const RangeSensorHardware::Parameter& parameter)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(hardware_name, id, parameter);
  // _shield->registerIotShieldRxDevice(range_sensor_hardware);
  _range_sensor_hardware[sensor_name] = range_sensor_hardware;
  return *this;
}                                               
  
HardwareComponentFactory&
HardwareComponentFactory::addImuSensor(const std::string& sensor_name, const std::string& hardware_name,
                                             const robot::ImuSensor::Parameter parameter)
{
  auto imu_hardware = std::make_shared<ImuSensorHardware>(hardware_name, parameter, _shield->getCommunicator());
  // _shield->registerIotShieldRxDevice(imu_hardware);
  _imu_sensor_hardware[sensor_name] = imu_hardware;
  return *this;
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
