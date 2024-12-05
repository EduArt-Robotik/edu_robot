/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/sensor_range.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>
#include <memory>

#include "edu_robot/hardware/communicator_node.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

class RangeSensorHardware : public SensorRange::SensorInterface
{
public:
  RangeSensorHardware(
    const std::uint8_t id, std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator);
  ~RangeSensorHardware() override = default;

  void initialize(const SensorRange::Parameter& parameter) override;

private:
  void processSending();

  std::uint8_t _id;
  std::shared_ptr<CommunicatorNode> _communication_node;  
};               

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
