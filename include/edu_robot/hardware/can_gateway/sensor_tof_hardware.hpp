/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can_gateway/can_gateway_device_interfaces.hpp"
#include "edu_robot/hardware/can_gateway/can/can_request.hpp"

#include <edu_robot/sensor_point_cloud.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class SensorTofHardware : public SensorPointCloud::SensorInterface
                        , public CanGatewayTxRxDevice
{
public:
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
    struct {
      std::uint32_t trigger = 0x388;
      std::uint32_t complete = 0x308;
      std::uint32_t measurement = 0x309;
    } can_id;
    std::uint8_t sensor_id = 1;
    bool trigger_measurement = true;
  };

  SensorTofHardware(
    const std::string& name, const Parameter& parameter, rclcpp::Node& ros_node,
    std::shared_ptr<Communicator> communicator);
  ~SensorTofHardware() override = default;

  void processRxData(const message::RxMessageDataBuffer& data) override;
  void initialize(const SensorPointCloud::Parameter& parameter) override;
  static Parameter get_parameter(const std::string& name, const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  void processMeasurement();

  const Parameter _parameter;
  std::shared_ptr<rclcpp::TimerBase> _timer_get_measurement;
  rclcpp::Node& _ros_node;

  struct {
    std::size_t number_of_zones;
    std::size_t current_zone;
    std::size_t next_expected_zone;
    std::vector<float> tan_x_lookup; // used to transform to point y
    std::vector<float> tan_y_lookup; // used to transform to point x    
    std::uint8_t frame_number;
    std::future<hardware::Request> future_response;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  } _processing_data;
};                              

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
