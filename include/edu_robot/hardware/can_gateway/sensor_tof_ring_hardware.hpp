/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/communicator_device_interfaces.hpp"
#include "edu_robot/hardware/can_gateway/sensor_tof_hardware.hpp"

#include <edu_robot/sensor_point_cloud.hpp>

#include <Eigen/Geometry>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class SensorTofRingHardware : public SensorPointCloud::SensorInterface
                            , public CommunicatorTxDevice
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

    inline std::size_t number_sensors() const { return tof_sensor.size(); };
  };

  SensorTofRingHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node,
    std::shared_ptr<Communicator> communicator);
  ~SensorTofRingHardware() override = default;

  void initialize(const SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(
    const std::string& name, const std::vector<std::string>& sensor_names, rclcpp::Node& ros_node);

private:
  void processStartMeasurement();
  void processPointcloudMeasurement(sensor_msgs::msg::PointCloud2& point_cloud, const std::size_t sensor_index);

  const Parameter _parameter;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
  rclcpp::Node& _ros_node;
  std::vector<std::shared_ptr<SensorTofHardware>> _sensor;

  struct {
    std::size_t current_sensor = 0;
    std::size_t next_expected_sensor = 0;
    std::uint8_t frame_number = 0;
    std::uint8_t* current_data_address = 0;
    std::uint8_t sensor_activation_bits = 0;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud; 
  } _processing_data;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
