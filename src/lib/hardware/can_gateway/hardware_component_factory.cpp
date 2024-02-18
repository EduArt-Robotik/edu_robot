#include "edu_robot/hardware/can_gateway/hardware_component_factory.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>

#include <functional>
#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

HardwareComponentFactory& HardwareComponentFactory::addLighting(const std::string& lighting_name)
{
  (void)lighting_name;
  // _lighting_hardware[lighting_name] = std::make_unique<ethernet::LightingHardware>(
  //   lighting_name, _shield->getCommunicator()
  // );

  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addMotorController(
  const std::string& controller_name, const std::uint32_t can_id_input, const std::uint32_t can_id_output)
{
  // \todo maybe make it configurable via ROS parameter
  MotorControllerHardware::Parameter parameter = {{can_id_input, can_id_output}};
  auto compound_motor = std::make_shared<MotorControllerHardware>(
    controller_name, parameter, _shield->getCommunicator(2)
  );
  _motor_controller_hardware.push_back(compound_motor);
  _shield->registerMotorControllerHardware(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofSensor(
  const std::string& sensor_name, const SensorTofHardware::Parameter& parameter, rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<SensorTofHardware>(
    sensor_name, parameter, ros_node, _shield->getCommunicator(1)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofRingSensor(
  const std::string& sensor_name, const SensorTofRingHardware::Parameter& parameter, rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter, ros_node, _shield->getCommunicator(1)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addImuSensor(
  const std::string& sensor_name, const std::uint32_t can_id, rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<ImuSensorHardware>(
    ros_node, can_id, _shield->getCommunicator(2)
  );

  return *this;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
