/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/sensor_imu.hpp>

#include <edu_robot/hardware/communicator_node.hpp>
#include <memory>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class ImuSensorHardware : public SensorImu::SensorInterface
{
public:
  ImuSensorHardware(std::shared_ptr<Executer> executer, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processRxData(const message::RxMessageDataBuffer& data);

  std::shared_ptr<CommunicatorNode> _communication_node;
  bool _raw_mode = true;
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
