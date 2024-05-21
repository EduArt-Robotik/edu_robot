#include "edu_robot/hardware/igus/hardware_component_factory.hpp"
#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"
#include "edu_robot/hardware/igus/can_gateway_shield.hpp"

#include <edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp>
#include <edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp>
#include <edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp>

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
  const std::string& controller_name, const MotorControllerHardware::Parameter& parameter)
{
  auto compound_motor = std::make_shared<MotorControllerHardware>(
    controller_name, parameter, _shield->getCommunicator(0)
  );
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
  const std::string& sensor_name, const std::uint32_t can_id)
{
  _hardware[sensor_name] = std::make_shared<hardware::can_gateway::ImuSensorHardware>(
    can_id, _shield->getCommunicator(0)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofSensor(
  const std::string& sensor_name, const hardware::can_gateway::SensorTofHardware::Parameter& parameter,
  rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<hardware::can_gateway::SensorTofHardware>(
    sensor_name, parameter, ros_node, _shield->getCommunicator(1));

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofRingSensor(
  const std::string& sensor_name, const hardware::can_gateway::SensorTofRingHardware::Parameter& parameter,
  rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<hardware::can_gateway::SensorTofRingHardware>(
    sensor_name, parameter, ros_node, _shield->getCommunicator(1)
  );

  return *this;
}

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
