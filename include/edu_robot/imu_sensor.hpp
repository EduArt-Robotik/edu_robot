/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/sensor.hpp"

#include <Eigen/Dense>

#include <rclcpp/clock.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace eduart {
namespace robot {

class ImuSensor : public Sensor
{
public:
  struct Parameter {
    bool raw_mode;
    std::string base_frame;
  };

  ImuSensor(const std::string& name, const std::string& frame_id, const tf2::Transform sensor_transform,
            const Parameter parameter, std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
            std::shared_ptr<rclcpp::Node> ros_node);
  ~ImuSensor() override = default;

protected:
  void processMeasurementData(const Eigen::Quaterniond& measurement);

private:
  const Parameter _parameter;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<rclcpp::Clock> _clock;
};

} // end namespace eduart
} // end namespace robot
