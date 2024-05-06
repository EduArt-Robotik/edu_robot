/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <edu_robot/sensor_range.hpp>
#include <edu_robot/hardware_component_interfaces.hpp>

#include "edu_robot/hardware/communicator_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class RangeSensorHardware : public SensorRange::SensorInterface
                          , public CommunicatorTxRxDevice
{
public:
  RangeSensorHardware(const std::uint8_t id, std::shared_ptr<Communicator> communicator);
  ~RangeSensorHardware() override = default;

  void initialize(const SensorRange::Parameter& parameter) override;

private:
  void processRxData(const message::RxMessageDataBuffer& data);
  void processMeasurement();

  std::uint8_t _id;
};               

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
