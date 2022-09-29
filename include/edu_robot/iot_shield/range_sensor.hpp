/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/range_sensor.hpp>

#include "edu_robot/iot_shield/iot_shield_device.hpp"

namespace eduart {
namespace robot {
namespace iotbot {

class RangeSensor : public eduart::robot::RangeSensor
                  , public IotShieldRxDevice
{
public:
  RangeSensor(const std::string& name, const std::string& frame_id, const std::string& reference_frame_id, 
              const tf2::Transform sensor_transform, const std::uint8_t id, const Parameter parameter,
              rclcpp::Node& ros_node);
  ~RangeSensor() override = default;

  void processRxData(const uart::message::RxMessageDataBuffer& data) override;
};               

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
