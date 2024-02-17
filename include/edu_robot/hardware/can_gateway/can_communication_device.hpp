/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communication_device.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class CanCommunicationDevice : public hardware::CommunicationDevice
{
public:
  CanCommunicationDevice(char const* const device_name);
  ~CanCommunicationDevice() override;

  void send(message::Byte const *const tx_buffer, const std::size_t length) override;
  message::RxMessageDataBuffer receive() override;

private:
  int _socket_fd = -1;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
