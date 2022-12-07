/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interface.hpp>
#include <edu_robot/imu_sensor.hpp>

#include "edu_robot/ethernet_gateway/ethernet_gateway_device.hpp"

namespace eduart {
namespace robot {
namespace ethernet {

class EthernetCommunicator;

class ImuSensorHardware : public robot::HardwareSensorInterface<Eigen::Quaterniond>
                        , public EthernetGatewayTxRxDevice
{
public:
  ImuSensorHardware(const std::string& hardware_name, const ImuSensor::Parameter parameter,
            std::shared_ptr<EthernetCommunicator> communicator);
  ~ImuSensorHardware() override = default;

  void processRxData(const tcp::message::RxMessageDataBuffer& data) override;
};

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
