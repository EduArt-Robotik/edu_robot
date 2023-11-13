#include "edu_robot/ethernet_gateway/hardware_component_factory.hpp"
#include "edu_robot/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/ethernet_gateway/ethernet_gateway_shield.hpp"

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace ethernet {

HardwareComponentFactory& HardwareComponentFactory::addLighting(
  const std::string& lighting_name, const std::string& hardware_name)
{
  _lighting_hardware[lighting_name] = std::make_unique<ethernet::LightingHardware>(
    hardware_name, _shield->getCommunicator()
  );

  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addMotorController(
  const std::string& motor_name, const std::string& hardware_name)
{
  auto compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_a", hardware_name + "_b", 0, _shield->getCommunicator()
  );
  _motor_controller_hardware[motor_name + "_a"] = compound_motor;
  _motor_controller_hardware[motor_name + "_b"] = compound_motor->dummyMotorController();
  _motor_sensor_hardware[motor_name + "_a"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_b"] = compound_motor->dummyMotorController();

  compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_c", hardware_name + "_d", 1, _shield->getCommunicator()
  );  
  _motor_controller_hardware[motor_name + "_c"] = compound_motor;
  _motor_controller_hardware[motor_name + "_d"] = compound_motor->dummyMotorController();

  _motor_sensor_hardware[motor_name + "_c"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_d"] = compound_motor->dummyMotorController();

  compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_e", hardware_name + "_f", 2, _shield->getCommunicator()
  );  
  _motor_controller_hardware[motor_name + "_e"] = compound_motor;
  _motor_controller_hardware[motor_name + "_f"] = compound_motor->dummyMotorController();

  _motor_sensor_hardware[motor_name + "_e"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_f"] = compound_motor->dummyMotorController();

  compound_motor = std::make_shared<CompoundMotorControllerHardware>(
    hardware_name + "_g", hardware_name + "_h", 2, _shield->getCommunicator()
  );  
  _motor_controller_hardware[motor_name + "_g"] = compound_motor;
  _motor_controller_hardware[motor_name + "_h"] = compound_motor->dummyMotorController();

  _motor_sensor_hardware[motor_name + "_g"] = compound_motor;
  _motor_sensor_hardware[motor_name + "_h"] = compound_motor->dummyMotorController();

  // _shield->registerIotShieldRxDevice(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addSingleChannelMotorController(
  const std::string& motor_name, const std::string& hardware_name)
{
  auto motor_controller = std::make_shared<SingleChannelMotorControllerHardware>(
    hardware_name + "_a", 0, _shield->getCommunicator()
  );
  _motor_controller_hardware[motor_name + "_a"] = motor_controller;
  _motor_sensor_hardware[motor_name + "_a"] = motor_controller;

  motor_controller = std::make_shared<SingleChannelMotorControllerHardware>(
    hardware_name + "_b", 1, _shield->getCommunicator()
  );
  _motor_controller_hardware[motor_name + "_b"] = motor_controller;
  _motor_sensor_hardware[motor_name + "_b"] = motor_controller;

  motor_controller = std::make_shared<SingleChannelMotorControllerHardware>(
    hardware_name + "_c", 2, _shield->getCommunicator()
  );
  _motor_controller_hardware[motor_name + "_c"] = motor_controller;
  _motor_sensor_hardware[motor_name + "_c"] = motor_controller;

  motor_controller = std::make_shared<SingleChannelMotorControllerHardware>(
    hardware_name + "_d", 3, _shield->getCommunicator()
  );    
  _motor_controller_hardware[motor_name + "_d"] = motor_controller;
  _motor_sensor_hardware[motor_name + "_d"] = motor_controller;

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addRangeSensor(
  const std::string& sensor_name, const std::string& hardware_name, const std::uint8_t id, rclcpp::Node& ros_node)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(
    hardware_name, id, ros_node, _shield->getCommunicator());
  // _shield->registerIotShieldRxDevice(range_sensor_hardware);
  _range_sensor_hardware[sensor_name] = range_sensor_hardware;
  return *this;
}                                               
  
HardwareComponentFactory& HardwareComponentFactory::addImuSensor(
  const std::string& sensor_name, const std::string& hardware_name, rclcpp::Node& ros_node)
{
  auto imu_hardware = std::make_shared<ImuSensorHardware>(
    hardware_name, ros_node, _shield->getCommunicator()
  );
  // _shield->registerIotShieldRxDevice(imu_hardware);
  _imu_sensor_hardware[sensor_name] = imu_hardware;
  return *this;
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
