/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"

#include <edu_robot/robot_hardware_interface.hpp>
#include <edu_robot/robot_status_report.hpp>
#include <edu_robot/processing_component/processing_component.hpp>

#include <memory>
#include <vector>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldCommunicator;
class IotShieldRxDevice;

class IotShield : public RobotHardwareInterface
                , public processing::ProcessingComponentOutput<float>
{
public:
  IotShield(char const* const device_name);
  ~IotShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<IotShieldCommunicator> getCommunicator() { return _communicator; }
  void registerIotShieldRxDevice(std::shared_ptr<IotShieldRxDevice> device);
  void processStatusReport();

private:
  std::shared_ptr<IotShieldCommunicator> _communicator;
  uart::message::TxMessageDataBuffer _tx_buffer;
  std::vector<std::shared_ptr<IotShieldRxDevice>> _rx_devices;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
