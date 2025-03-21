#include "edu_robot/hardware/can_gateway/hardware_component_factory.hpp"
#include "edu_robot/hardware/can_gateway/sensor_point_cloud_fusion.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/can_gateway/range_sensor_virtual.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_ring_hardware.hpp"
#include "edu_robot/hardware/can_gateway/sensor_point_cloud_fusion.hpp"

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>

#include <memory>
#include <rclcpp/node.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

HardwareComponentFactory& HardwareComponentFactory::addLighting()
{
  const std::array<std::string, 5> lighting_name = {"all", "head", "back", "left_side", "right_side"};

  LightingHardwareManager::instance().initialize(
    _shield->getExecuter(), _shield->getCommunicator(2), _shield->getCommunicator(1)
  );

  for (const auto& name : lighting_name) {
    _hardware[name] = LightingHardwareManager::instance().lighting(name);
  }

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
  const std::string& sensor_name, const std::vector<std::string>& left_ring_sensors,
  const std::vector<std::string>& right_ring_sensors, rclcpp::Node& ros_node)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  const auto parameter_left_sensors = SensorTofRingHardware::get_parameter(
    sensor_name + "_left", left_ring_sensors, ros_node
  );
  const auto parameter_right_sensors = SensorTofRingHardware::get_parameter(
    sensor_name + "_right", right_ring_sensors, ros_node
  );

  auto left_ring = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter_left_sensors, ros_node, _shield->getExecuter(), _shield->getCommunicator(2)
  );
  auto right_ring = std::make_shared<SensorTofRingHardware>(
    sensor_name, parameter_right_sensors, ros_node, _shield->getExecuter(), _shield->getCommunicator(1)
  );
  std::vector<std::shared_ptr<SensorPointCloud::SensorInterface>> ring = { left_ring, right_ring };

  _hardware[sensor_name] = std::make_shared<SensorPointCloudFusion>(ring);
  _hardware.insert(left_ring->virtualRangeSensor().begin(), left_ring->virtualRangeSensor().end());
  _hardware.insert(right_ring->virtualRangeSensor().begin(), right_ring->virtualRangeSensor().end());

  return *this;
}

// HardwareComponentFactory& HardwareComponentFactory::addRangeSensor(
//   const std::string& sensor_name, const std::uint8_t id)
// {
//   auto range_sensor_hardware = std::make_shared<RangeSensorHardware>(
//     id, _shield->getExecuter(), _shield->getCommunicator(0)
//   );
//   // _shield->registerIotShieldRxDevice(range_sensor_hardware);
//   _hardware[sensor_name] = range_sensor_hardware;
//   return *this;
// } 

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
