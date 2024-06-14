/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/sensor_range.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>

#include <edu_robot/hardware/communicator_node.hpp>

#include <rclcpp/parameter.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class RangeSensorHardware : public SensorRange::SensorInterface
                          , public CommunicatorRxNode
{
public:
  RangeSensorHardware(const std::uint8_t id, std::shared_ptr<Communicator> communicator);
  ~RangeSensorHardware() override = default;

  void processRxData(const message::RxMessageDataBuffer& data);
  void initialize(const SensorRange::Parameter& parameter) override;

private:
  std::uint8_t _id;
};               

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
