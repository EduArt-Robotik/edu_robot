#include "edu_robot/hardware/can_gateway/hardware_component_factory.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"
#include "edu_robot/hardware/can_gateway/motor_controller_hardware.hpp"
#include "edu_robot/hardware/can_gateway/imu_sensor_hardware.hpp"
#include "edu_robot/hardware/can_gateway/lighting_hardware.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_sensor_ring_adapter.hpp"
#include "edu_robot/hardware/can_gateway/sensor_virtual_range_adapter.hpp"

#include <sensorring/SensorRingFactory.hpp>
#include <sensorring/manager/MeasurementManager.hpp>
#include <sensorring/manager/ManagerParams.hpp>
#include <sensorring/math/Math.hpp>

#include <edu_robot/hardware/can_gateway/can_gateway_shield.hpp>
#include <edu_robot/sensor.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

HardwareComponentFactory& HardwareComponentFactory::addLighting()
{
  if (!_measurement_manager) {
    throw std::logic_error(
      "HardwareComponentFactory::addLighting(): addSensorRing() must be called before addLighting()");
  }

  const std::size_t total = _measurement_manager->lights().size();

  std::unordered_map<std::string, std::vector<std::size_t>> zone_map;

  // Ensure all zone keys exist (LightingHardwareManager throws on unknown zones).
  zone_map["all"];
  zone_map["head"];
  zone_map["back"];
  zone_map["left_side"];
  zone_map["right_side"];

  // "all" zone always contains every light.
  for (std::size_t i = 0; i < total; ++i) {
    zone_map["all"].push_back(i);
  }

  // Build per-zone mappings by iterating all lights.
  std::size_t light_idx = 0;
  auto lights = _measurement_manager->lights();
  for (const auto& light : lights) {
    const std::string side = (light.getBoardContext()->orientation ==
                              eduart::sensorring::board::Orientation::Left)
                                 ? "left_side"
                                 : "right_side";
    zone_map[side].push_back(light_idx);

    switch (light.getBoardContext()->board_type) {
    case eduart::sensorring::board::SensorBoardType::Headlight:
      zone_map["head"].push_back(light_idx);
      break;
    case eduart::sensorring::board::SensorBoardType::Taillight:
      zone_map["back"].push_back(light_idx);
      break;
    default:
      break;
    }
    ++light_idx;
  }

  auto lighting_manager = std::make_shared<LightingHardwareManager>(_measurement_manager, std::move(zone_map));

  const std::array<std::string, 5> lighting_names = {"all", "head", "back", "left_side", "right_side"};
  for (const auto& name : lighting_names) {
    _hardware[name] = std::make_shared<LightingGroup>(name, lighting_manager);
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

HardwareComponentFactory& HardwareComponentFactory::addSensorRing(
  const std::string& sensor_name, const std::vector<std::string>& left_sensor_names,
  const std::vector<std::string>& right_sensor_names, rclcpp::Node& ros_node)
{
  sensorring::SensorRingFactory factory(sensorring::ValidationMode::Relaxed);

  // Disable thermal sensor on Headlights
  sensorring::device::HTPA32_Params default_htpa32_params;
  default_htpa32_params.enable = false;
  factory.setDefaultDeviceParams(default_htpa32_params);

  // Collect all board transforms (declared once via get_transform_from_parameter) so they can be
  // reused later for virtual range sensors without triggering ParameterAlreadyDeclaredException.
  std::vector<tf2::Transform> all_transforms;

  // Left ring → eduart-can1 (communicator index 1, added first so lights[0..n-1])
  factory.addInterface({ sensorring::com::InterfaceType::SocketCan, "eduart-can1" });
  const std::string left_prefix = sensor_name + "_left";
  for (const auto& board_name : left_sensor_names) {
    const tf2::Transform tf = Sensor::get_transform_from_parameter(
      left_prefix + '.' + board_name, ros_node);
    all_transforms.push_back(tf);

    tf2::Matrix3x3 rot_matrix(tf.getRotation());
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);

    sensorring::board::SensorBoardParams board;
    board.translation = { tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z() };
    board.rotation    = {
      sensorring::math::radiansToDegrees(roll),
      sensorring::math::radiansToDegrees(pitch),
      sensorring::math::radiansToDegrees(yaw)
    };

    // Require at least one Headlight and a Taillight. Factory ValidationMode is "Relaxed", so order doesn't matter.
    if(board_name == "front") {
      board.board_type = sensorring::board::SensorBoardType::Headlight;
    } else if(board_name == "rear"){
      board.board_type = sensorring::board::SensorBoardType::Taillight;
    }

    factory.expectBoard(board);
  }

  // Right ring → eduart-can0 (communicator index 0, added second so lights[n..])
  factory.addInterface({ sensorring::com::InterfaceType::SocketCan, "eduart-can0" });
  const std::string right_prefix = sensor_name + "_right";
  for (const auto& board_name : right_sensor_names) {
    const tf2::Transform tf = Sensor::get_transform_from_parameter(
      right_prefix + '.' + board_name, ros_node);
    all_transforms.push_back(tf);

    tf2::Matrix3x3 rot_matrix(tf.getRotation());
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);

    sensorring::board::SensorBoardParams board;
    board.translation = { tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z() };
    board.rotation    = {
      sensorring::math::radiansToDegrees(roll),
      sensorring::math::radiansToDegrees(pitch),
      sensorring::math::radiansToDegrees(yaw)
    };

    // Require at least one Headlight and a Taillight. Factory ValidationMode is "Relaxed", so order doesn't matter.
    if(board_name == "front") {
      board.board_type = sensorring::board::SensorBoardType::Headlight;
    } else if(board_name == "rear"){
      board.board_type = sensorring::board::SensorBoardType::Taillight;
    }

    factory.expectBoard(board);
  }

  sensorring::manager::ManagerParams params;
  _measurement_manager = std::make_shared<sensorring::manager::MeasurementManager>(params, factory);

  const std::size_t total_sensors = left_sensor_names.size() + right_sensor_names.size();
  _hardware[sensor_name] = std::make_shared<SensorTofSensorRingAdapter>(
    _measurement_manager, total_sensors);

  // Virtual range sensors — check per-board ROS parameter flag.
  // Iterate over all_prefixes (known at config time) rather than depthSensors(),
  // which may return an empty vector before the CAN ring has been discovered.
  const std::vector<std::string> all_sensor_names = [&] {
    std::vector<std::string> combined;
    combined.insert(combined.end(), left_sensor_names.begin(), left_sensor_names.end());
    combined.insert(combined.end(), right_sensor_names.begin(), right_sensor_names.end());
    return combined;
  }();
  const std::vector<std::string> all_prefixes = [&] {
    std::vector<std::string> combined;
    for (const auto& name : left_sensor_names)  { combined.push_back(left_prefix  + '.' + name); }
    for (const auto& name : right_sensor_names) { combined.push_back(right_prefix + '.' + name); }
    return combined;
  }();

  for (std::size_t i = 0; i < all_prefixes.size(); ++i) {
    const std::string param_path = all_prefixes[i] + ".virtual_range_sensor";
    ros_node.declare_parameter<bool>(param_path, false);
    const bool want_virtual_range = ros_node.get_parameter(param_path).as_bool();

    if (want_virtual_range) {
      // Reuse the transform already read above — do NOT call get_transform_from_parameter again,
      // as that would re-declare the same parameters and throw ParameterAlreadyDeclaredException.
      const std::string side = (i < left_sensor_names.size()) ? "left" : "right";
      const std::string range_key = "range/" + all_sensor_names[i] + "/" + side;
      _hardware[range_key] = std::make_shared<SensorVirtualRangeAdapter>(
        _measurement_manager, static_cast<unsigned int>(i), all_transforms[i]);
    }
  }

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
