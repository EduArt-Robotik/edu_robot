/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/sensor.hpp"

#include <functional>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/range.hpp>

#include <memory>
#include <string>

namespace eduart {
namespace robot {

/**
 * \brief Represents a range sensor without a concrete hardware realization, that has to be implemented. This class
 *        creates an publisher for publishing its measurements. This class needs to be realized by a specific
 *        hardware layer.
 *
 * \todo below is only a draft, a first try --> please review concept after first release
 */
class RangeSensor : public Sensor
{
public:
  struct Parameter {
    float field_of_view;
    float range_min;
    float range_max;
  };

  RangeSensor(const std::string& name, const std::string& frame_id, const tf2::Transform sensor_transform,
              const Parameter parameter, std::shared_ptr<rclcpp::Node> ros_node);

  ~RangeSensor() override = default;

protected:
  void processMeasurementData(const float measurement);

private:
  const Parameter _parameter;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Range>> _publisher;
  std::shared_ptr<rclcpp::Clock> _clock;
};

} // end namespace eduart
} // end namespace robot