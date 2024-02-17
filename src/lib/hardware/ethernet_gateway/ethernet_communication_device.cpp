#include "edu_robot/hardware/ethernet_gateway/ethernet_communication_device.hpp"

#include <edu_robot/hardware_error.hpp>

#include <rclcpp/logging.hpp>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <net/if.h>

#include <arpa/inet.h>

#include <cstring>
#include <unistd.h>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {

EthernetCommunicationDevice::EthernetCommunicationDevice(char const* const ip_address, const std::uint16_t port)
{
  // Creating Socket Instance
  sockaddr_in socket_address;

  socket_address.sin_addr.s_addr = inet_addr(ip_address);
	socket_address.sin_family = AF_INET;
	socket_address.sin_port = htons( port );
  // _socket_fd = ::socket(AF_INET , SOCK_STREAM , 0);
  _socket_fd = ::socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP);

  if (_socket_fd < -1) {
    throw HardwareError(State::UDP_SOCKET_ERROR, "Error occurred during opening TCP socket.");
  }
  if (::connect(_socket_fd, (struct sockaddr*)&socket_address, sizeof(socket_address)) < 0) {
    ::close(_socket_fd);
    throw HardwareError(State::UDP_SOCKET_ERROR, "Error occurred during opening TCP socket.");
  }
}

EthernetCommunicationDevice::~EthernetCommunicationDevice()
{
  ::close(_socket_fd);
}

void EthernetCommunicationDevice::send(message::Byte const *const tx_buffer, const std::size_t length)
{
  // Debugging
  std::stringstream debug_out;
  debug_out << "sending: " << std::hex;

  for (std::size_t i = 0; i < length; ++i) {
    debug_out << static_cast<int>(tx_buffer[i]) << " ";
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EthernetCommunicationDevice"), debug_out.str());

  // Sending Given Data
  if (::send(_socket_fd, tx_buffer, length, 0) != static_cast<int>(length)) {
    throw HardwareError(State::UDP_SOCKET_ERROR, "Can't write data to interface.");
  }
}

message::RxMessageDataBuffer EthernetCommunicationDevice::receive()
{
  // Receiving Data from Interface
  std::uint8_t buffer[_max_rx_buffer_queue_size];

  const int received_bytes = ::recv(_socket_fd, buffer, _max_rx_buffer_queue_size, MSG_DONTWAIT);

  if (received_bytes < 0) {
    return { };
  }

  // Set correct size to rx buffer.
  message::RxMessageDataBuffer rx_buffer;

  rx_buffer.resize(received_bytes);
  std::copy(std::begin(buffer), std::begin(buffer) + received_bytes, rx_buffer.begin());

  // Debugging
  std::stringstream debug_out;
  debug_out << "received: " << std::hex;

  for (const auto byte : rx_buffer) {
    debug_out << static_cast<int>(byte) << " ";
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("EthernetCommunicationDevice"), debug_out.str());

  return rx_buffer;
}

} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
