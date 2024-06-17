#include "edu_robot/hardware/can_gateway/hardware_component_factory.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/can_gateway/range_sensor_hardware.hpp"

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>

#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

HardwareComponentFactory& HardwareComponentFactory::addLighting(const std::string& lighting_name)
{
  _hardware[lighting_name] = std::make_unique<can_gateway::LightingHardware>(
    lighting_name, _shield->getExecuter(), _shield->getCommunicator(0)
  );

  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addMotorController(
  const std::string& controller_name, const MotorControllerHardware::Parameter& parameter)
{
  // \todo maybe make it configurable via ROS parameter
  auto compound_motor = std::make_shared<MotorControllerHardware>(
    controller_name, parameter, _shield->getExecuter(), _shield->getCommunicator(0)
  );
  _motor_controller_hardware.push_back(compound_motor);
  _shield->registerMotorControllerHardware(compound_motor);

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofSensor(
  const std::string& sensor_name, const SensorTofHardware::Parameter& parameter, rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<SensorTofHardware>(
    sensor_name, parameter, ros_node, _shield->getExecuter(), _shield->getCommunicator(1)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofRingSensor(
  const std::string& sensor_name, const SensorTofRingHardware::Parameter& parameter, rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter, ros_node, _shield->getExecuter(), _shield->getCommunicator(1)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addRangeSensor(
  const std::string& sensor_name, const std::uint8_t id)
{
  auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(
    id, _shield->getExecuter(), _shield->getCommunicator(0)
  );
  // _shield->registerIotShieldRxDevice(range_sensor_hardware);
  _hardware[sensor_name] = range_sensor_hardware;
  return *this;
} 

HardwareComponentFactory& HardwareComponentFactory::addImuSensor(
  const std::string& sensor_name, const std::uint32_t can_id)
{
  _hardware[sensor_name] = std::make_shared<ImuSensorHardware>(
    can_id, _shield->getExecuter(), _shield->getCommunicator(0)
  );

  return *this;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
