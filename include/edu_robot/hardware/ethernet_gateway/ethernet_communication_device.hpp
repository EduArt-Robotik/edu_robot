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
namespace ethernet {

class EthernetCommunicationDevice : public hardware::CommunicationDevice
{
public:
  EthernetCommunicationDevice(char const* const ip_address, const std::uint16_t port);
  ~EthernetCommunicationDevice() override;

  void send(message::Byte const *const tx_buffer, const std::size_t length) override;
  message::RxMessageDataBuffer receive() override;

private:
  int _socket_fd = -1;

  inline static constexpr std::size_t _max_rx_buffer_queue_size = 100;
};

} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
