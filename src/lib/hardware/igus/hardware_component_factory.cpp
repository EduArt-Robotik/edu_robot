#include "edu_robot/hardware/igus/hardware_component_factory.hpp"
#include "edu_robot/hardware/igus/motor_controller_hardware.hpp"
#include "edu_robot/hardware/igus/can_gateway_shield.hpp"

#include <edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp>
#include <edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp>
#include <edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp>
#include <edu_robot/hardware/can_gateway/sensor_point_cloud_fusion.hpp>

#include <rclcpp/node.hpp>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {

using can_gateway::SensorTofRingHardware;
using can_gateway::SensorPointCloudFusion;

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
  auto motor_controller = std::make_shared<MotorControllerHardware>(
    controller_name, parameter, _shield->getExecuter(0), _shield->getCommunicator(0)
  );
  _motor_controller_hardware.push_back(motor_controller);
  _shield->registerMotorControllerHardware(motor_controller);

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
    can_id,  _shield->getExecuter(1), _shield->getCommunicator(0)
  );

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofSensor(
  const std::string& sensor_name, const hardware::can_gateway::SensorTofHardware::Parameter& parameter,
  rclcpp::Node& ros_node)
{
  _hardware[sensor_name] = std::make_shared<hardware::can_gateway::SensorTofHardware>(
    sensor_name, parameter, ros_node,  _shield->getExecuter(2), _shield->getCommunicator(1));

  return *this;
}

HardwareComponentFactory& HardwareComponentFactory::addTofRingSensor(
  const std::string& sensor_name, const std::vector<std::string>& left_ring_sensors,
  const std::vector<std::string>& right_ring_sensors, rclcpp::Node& ros_node)
{
  const auto parameter_left_sensors = SensorTofRingHardware::get_parameter(
    sensor_name + "_left", left_ring_sensors, ros_node
  );
  const auto parameter_right_sensors = SensorTofRingHardware::get_parameter(
    sensor_name + "_right", right_ring_sensors, ros_node
  );

  auto left_ring = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter_left_sensors, ros_node, _shield->getExecuter(2), _shield->getCommunicator(1)
  );
  auto right_ring = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter_right_sensors, ros_node, _shield->getExecuter(3), _shield->getCommunicator(2)
  );
  std::vector<std::shared_ptr<SensorPointCloud::SensorInterface>> ring = { left_ring, right_ring };

  _hardware[sensor_name] = std::make_shared<SensorPointCloudFusion>(ring);

  return *this;
}

} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
