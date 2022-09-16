/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart_message_conversion.hpp"
#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/robot_hardware_interface.hpp"
#include "edu_robot/robot_status_report.hpp"

#include <memory>
#include <array>

namespace eduart {
namespace robot {
namespace iot_bot {

class IotShieldCommunicator;

class IotShield : public RobotHardwareInterface
{
public:
  IotShield(char const* const device_name);
  ~IotShield() override;
  void enable() override;
  void disable() override;
  RobotStatusReport getStatusReport() override;

  std::shared_ptr<IotShieldCommunicator> getCommunicator() { return _communicator; }

private:
  void processStatusReport(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer);

  std::shared_ptr<IotShieldCommunicator> _communicator;
  std::array<std::uint8_t, UART::BUFFER::TX_SIZE> _tx_buffer;
  RobotStatusReport _report;

#if _WITH_MRAA
   std::unique_ptr<mraa::Uart> _uart;
#endif
};

} // end namespace iot_bot
} // end namespace eduart
} // end namespace robot
