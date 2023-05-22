/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include "edu_robot/sensor.hpp"

#include <Eigen/Dense>

#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

namespace eduart {
namespace robot {

class ImuSensor : public Sensor
{
public:
  struct Parameter {
    bool raw_data_mode = false;
    bool publish_tf = true;
    float fusion_weight = 0.03f;
    struct {
      // Use mounting orientation from IoT Shield as default.
      float roll  = -90.0f * M_PI / 180.0f;
      float pitch = -90.0f * M_PI / 180.0f;
      float yaw   = -90.0f * M_PI / 180.0f;
    } mount_orientation;
    std::string rotated_frame = "imu/rotated";
  };

  using SensorInterface = HardwareSensorInterface<Parameter, Eigen::Quaterniond, Eigen::Vector3d, Eigen::Vector3d>;

  ImuSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id,
            const tf2::Transform sensor_transform, const Parameter parameter,
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, rclcpp::Node& ros_node,
            std::shared_ptr<SensorInterface> hardware_interface);
  ~ImuSensor() override = default;

  static ImuSensor::Parameter get_parameter(
    const std::string& name, const ImuSensor::Parameter& default_parameter, rclcpp::Node& ros_node);

protected:
  void processMeasurementData(
    const Eigen::Quaterniond& measurement, const Eigen::Vector3d& angular_velocity,
    const Eigen::Vector3d& linear_acceleration);

private:
  const Parameter _parameter;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> _pub_imu_message;
  std::shared_ptr<rclcpp::Clock> _clock;
  std::shared_ptr<SensorInterface> _hardware_interface;
};

} // end namespace eduart
} // end namespace robot
