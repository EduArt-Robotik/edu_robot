/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"

#include <edu_robot/hardware/communicator_node.hpp>
#include <edu_robot/hardware/message_buffer.hpp>

#include <edu_robot/sensor_point_cloud.hpp>

#include <Eigen/Geometry>

#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class SensorTofRingHardware : public SensorPointCloud::SensorInterface
{
public:
  struct Parameter {
    struct TofSensor {
      SensorTofHardware::Parameter parameter;
      std::string name;
      tf2::Transform transform;
    };
    std::vector<TofSensor> tof_sensor;
    std::chrono::milliseconds measurement_interval{100};
    std::uint32_t can_id_trigger = 0x388;
    std::uint32_t can_id_measurement_complete = 0x308;

    inline std::size_t number_sensors() const { return tof_sensor.size(); };
  };

  SensorTofRingHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, std::shared_ptr<Executer> executer,
    std::shared_ptr<Communicator> communicator);
  ~SensorTofRingHardware() override = default;

  void initialize(const SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(
    const std::string& name, const std::vector<std::string>& sensor_names, rclcpp::Node& ros_node);
  inline const Parameter& parameter() const { return _parameter; }

private:
  void processStartMeasurement();
  void processFinishMeasurement(std::uint8_t sensor_id);    // Gets called by individual Sensors when measurement is finished
  void processPointcloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index);

  const Parameter _parameter;
  rclcpp::Node& _ros_node;
  std::shared_ptr<CommunicatorNode> _communication_node;
  std::vector<std::shared_ptr<SensorTofHardware>> _sensor;

  struct {
    std::uint8_t frame_number = 0;
    std::uint16_t sensor_activation_bits = 0;
    std::uint16_t active_measurement = 0;
    std::vector<bool> received_points;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  } _processing_data;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
