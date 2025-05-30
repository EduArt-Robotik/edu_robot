/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communicator_node.hpp>

#include <edu_robot/sensor_point_cloud.hpp>

#include "edu_robot/hardware/can_gateway/sensor_virtual_range.hpp"

#include <cstddef>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class SensorTofHardware : public SensorPointCloud::SensorInterface
{
public:
  using MeasurementCompleteCallback = std::function<void()>;
  struct Parameter {
    struct {
      std::size_t vertical = 8;
      std::size_t horizontal = 8;
    } number_of_zones; // number of sensor zones (pixel)
    struct {
      Angle vertical = Angle::createFromDegree(45);
      Angle horizontal = Angle::createFromDegree(45);
    } fov;
    std::chrono::milliseconds measurement_interval{100};    
    std::uint8_t sensor_id = 1;
    bool trigger_measurement = true;
  };

  SensorTofHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node, std::shared_ptr<Executer> executer,
    std::shared_ptr<Communicator> communicator, std::shared_ptr<SensorVirtualRange> virtual_range_sensor = nullptr);
  ~SensorTofHardware() override = default;

  void initialize(const SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);
  inline void registerMeasurementCompleteCallback(MeasurementCompleteCallback callback){
    _callback_finished_measurement = callback;
  }

private:
  void processRxData(const message::RxMessageDataBuffer& data);
  void processMeasurement();

  const Parameter _parameter;
  rclcpp::Node& _ros_node;
  std::shared_ptr<CommunicatorNode> _communication_node;
  MeasurementCompleteCallback _callback_finished_measurement = nullptr;
  std::shared_ptr<SensorVirtualRange> _virtual_range_sensor = nullptr;

  struct {
    std::uint32_t trigger = 0x388;
    std::uint32_t complete = 0x308;
    std::uint32_t measurement = 0x309;
  } _can_id;   

  struct { 
    std::size_t number_of_zones;
    std::size_t current_zone;
    std::size_t next_expected_zone;
    std::vector<float> tan_x_lookup; // used to transform to point y
    std::vector<float> tan_y_lookup; // used to transform to point x    
    std::uint8_t frame_number;
    std::size_t point_counter = 0;
    std::size_t point_index = 0;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  } _processing_data;
};                              

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
