/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communication_device.hpp>
#include <edu_robot/hardware/message_buffer.hpp>

#if _WITH_MRAA
#include <memory>

#include <mraa/common.hpp>
#include <mraa/uart.hpp>
#endif

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class UartCommunicationDevice : public hardware::CommunicationDevice
{
public:
  UartCommunicationDevice(char const* const device_name);
  ~UartCommunicationDevice() override;

  void send(message::Byte const *const tx_buffer, const std::size_t length) override;
  message::RxMessageDataBuffer receive() override;

private:
#if _WITH_MRAA
  std::unique_ptr<mraa::Uart> _uart;
#endif
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
