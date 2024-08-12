/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>
#include <memory>

#include "edu_robot/hardware/communicator_node.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

class ImuSensorHardware : public SensorImu::SensorInterface
{
public:
  ImuSensorHardware(std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processSending();

  std::shared_ptr<CommunicatorNode> _communication_node;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
