#include "edu_robot/hardware/ethernet_gateway/hardware_component_factory.hpp"
#include "edu_robot/hardware/ethernet_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/range_sensor_hardware.hpp"
#include "edu_robot/hardware/ethernet_gateway/ethernet_gateway_shield.hpp"

#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

HardwareComponentFactory& HardwareComponentFactory::addLighting(const std::string& lighting_name)
{
  _hardware[lighting_name] = std::make_unique<ethernet::LightingHardware>(
    lighting_name, _shield->getCommunicator()
  );

  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addMotorController(
  const std::string& controller_name, const MotorControllerHardware<2>::Parameter& parameter)
{
  auto compound_motor = std::make_shared<MotorControllerHardware<2>>(
    controller_name, parameter, _shield->getCommunicator());
  _motor_controller_hardware.push_back(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addSingleChannelMotorController(
  const std::string& controller_name, const MotorControllerHardware<1>::Parameter& parameter)
{
  auto compound_motor = std::make_shared<MotorControllerHardware<1>>(
    controller_name, parameter, _shield->getCommunicator());
  _motor_controller_hardware.push_back(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addRangeSensor(
  const std::string& sensor_name, const std::uint8_t id, rclcpp::Node& ros_node)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(
    id, ros_node, _shield->getCommunicator()
  );
  // _shield->registerIotShieldRxDevice(range_sensor_hardware);
  _hardware[sensor_name] = range_sensor_hardware;
  return *this;
}                                               
  
HardwareComponentFactory& HardwareComponentFactory::addImuSensor(
  const std::string& sensor_name, rclcpp::Node& ros_node)
{
  auto imu_hardware = std::make_shared<ImuSensorHardware>(
    ros_node, _shield->getCommunicator()
  );
  // _shield->registerIotShieldRxDevice(imu_hardware);
  _hardware[sensor_name] = imu_hardware;
  return *this;
}

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
