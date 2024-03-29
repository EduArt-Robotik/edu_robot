#include "edu_robot/hardware/igus/hardware_component_factory.hpp"
#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"

#include <edu_robot/hardware/igus/can_gateway_shield.hpp>

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

HardwareComponentFactory& HardwareComponentFactory::addLighting(const std::string& lighting_name)
{
  (void)lighting_name;
  // _lighting_hardware[lighting_name] = std::make_unique<ethernet::LightingHardware>(
  //   lighting_name, _shield->getCommunicator()
  // );

  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addMotorController(
  const std::string& controller_name, const std::size_t can_id)
{
  auto compound_motor = std::make_shared<MotorControllerHardware>(controller_name, can_id, _shield->getCommunicator());
  _motor_controller_hardware.push_back(compound_motor);
  _shield->registerMotorControllerHardware(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addRangeSensor(
  const std::string& sensor_name, const std::uint8_t id, rclcpp::Node& ros_node)
{
  (void)sensor_name;
  (void)id;
  (void)ros_node;
  // auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(
  //   id, ros_node, _shield->getCommunicator()
  // );
  // // _shield->registerIotShieldRxDevice(range_sensor_hardware);
  // _range_sensor_hardware[sensor_name] = range_sensor_hardware;
  return *this;
}                                               
  
HardwareComponentFactory& HardwareComponentFactory::addImuSensor(
  const std::string& sensor_name, rclcpp::Node& ros_node)
{
  (void)sensor_name;
  (void)ros_node;
  // auto imu_hardware = std::make_shared<ImuSensorHardware>(
  //   ros_node, _shield->getCommunicator()
  // );
  // // _shield->registerIotShieldRxDevice(imu_hardware);
  // _imu_sensor_hardware[sensor_name] = imu_hardware;
  return *this;
}

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
