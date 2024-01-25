/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interfaces.hpp"
#include "edu_robot/sensor.hpp"
#include "edu_robot/diagnostic/standard_deviation.hpp"

#include <Eigen/Dense>

#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

namespace eduart {
namespace robot {

class SensorImu : public Sensor
{
public:
  struct Parameter {
    bool raw_data_mode = false;
    bool publish_tf = true;
    bool publish_orientation_tf = false;
    float fusion_weight = 0.03f;
    struct {
      // Use mounting orientation from IoT Shield as default.
      float roll  = -90.0f * M_PI / 180.0f;
      float pitch = -90.0f * M_PI / 180.0f;
      float yaw   = -90.0f * M_PI / 180.0f;
    } mount_orientation;
    std::string rotated_frame = "imu/rotated";
  };

  class SensorInterface : public HardwareComponent<Parameter>
                        , public HardwareSensor<Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d>
  { };

  SensorImu(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
            const tf2::Transform sensor_transform, const Parameter parameter,
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, rclcpp::Node& ros_node,
            std::shared_ptr<SensorInterface> hardware_interface);
  ~SensorImu() override = default;

  static SensorImu::Parameter get_parameter(
    const std::string& name, const SensorImu::Parameter& default_parameter, rclcpp::Node& ros_node);
  inline const Parameter& parameter() const { return _parameter; }

protected:
  void processMeasurementData(
    const Eigen::Quaterniond& measurement, const Eigen::Vector3d& angular_velocity,
    const Eigen::Vector3d& linear_acceleration);

private:
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  const Parameter _parameter;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> _pub_imu_message;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;

  // diagnostic
  rclcpp::Time _last_processing;
  std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>> _processing_dt_statistic;
};

} // end namespace eduart
} // end namespace robot
