/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/communication_device.hpp>
#include <edu_robot/hardware/message_buffer.hpp>

#include <string>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

/**
 * \brief Network-based communication device for debugging purposes.
 *        Implements CommunicationDevice interface using TCP/IP sockets.
 *        Can be used as a drop-in replacement for UartCommunicationDevice for example.
 */
class NetworkCommunicationDevice : public hardware::CommunicationDevice
{
public:
  /**
   * \brief Constructs a network communication device and connects to the specified host and port.
   * \param host Hostname or IP address to connect to
   * \param port Port number to connect to
   * \param max_message_size Maximum size of messages to be received
   * \throws HardwareError if connection fails
   */
  NetworkCommunicationDevice(const std::string& host, const int port, const std::size_t max_message_size = 32);
  ~NetworkCommunicationDevice() override;

  void send(message::Byte const *const tx_buffer, const std::size_t length) override;
  message::RxMessageDataBuffer receive() override;

private:
  int _socket_fd;
  std::string _host;
  int _port;
  std::size_t _max_message_size;
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
