#include "edu_robot/hardware/network_communication_device.hpp"

#include <edu_robot/hardware_error.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

using namespace std::chrono_literals;

NetworkCommunicationDevice::NetworkCommunicationDevice(
  const std::string& host, const int port, const std::size_t max_message_size)
  : _socket_fd(-1)
  , _host(host)
  , _port(port)
  , _max_message_size(max_message_size)
{
  RCLCPP_INFO(
    rclcpp::get_logger("NetworkCommunicationDevice"), 
    "Connecting to network device %s:%d for UART debugging.", host.c_str(), port
  );
  
  // Create socket
  if ((_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    throw HardwareError(
      State::UART_HARDWARE_ERROR, "NetworkCommunicationDevice: failed to create socket."
    );
  }

  // Resolve hostname
  struct hostent* server = gethostbyname(host.c_str());

  if (server == nullptr) {
    // no such host found or available
    close(_socket_fd);
    throw HardwareError(
      State::UART_HARDWARE_ERROR, "NetworkCommunicationDevice: failed to resolve hostname " + host
    );
  }

  // Setup server address
  struct sockaddr_in server_addr;
  std::memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  std::memcpy(&server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
  server_addr.sin_port = htons(port);

  // Connect to server
  if (connect(_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    close(_socket_fd);
    throw HardwareError(
      State::UART_HARDWARE_ERROR, "NetworkCommunicationDevice: failed to connect to " + host + ":" + std::to_string(port)
    );
  }

  // Set socket to non-blocking mode for receive timeout
  int flags = fcntl(_socket_fd, F_GETFL, 0);
  fcntl(_socket_fd, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO(rclcpp::get_logger("NetworkCommunicationDevice"), "Successfully connected to %s:%d", host.c_str(), port);
}

NetworkCommunicationDevice::~NetworkCommunicationDevice()
{
  if (_socket_fd >= 0) {
    RCLCPP_INFO(rclcpp::get_logger("NetworkCommunicationDevice"), "Closing network connection to %s:%d", _host.c_str(), _port);
    close(_socket_fd);
    _socket_fd = -1;
  }
}

void NetworkCommunicationDevice::send(message::Byte const *const tx_buffer, const std::size_t length)
{
  if (_socket_fd < 0) {
    throw HardwareError(State::UART_SENDING_FAILED, "NetworkCommunicationDevice: socket not connected.");
  }

  const char* data = static_cast<const char*>(static_cast<const void*>(tx_buffer));
  std::size_t total_sent = 0;

  // try to send as one package
  ssize_t sent_bytes = ::send(_socket_fd, data + total_sent, length - total_sent, 0);
    
  if (sent_bytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Socket buffer full, wait a bit and retry
      throw HardwareError(
        State::UART_SENDING_FAILED, "NetworkCommunicationDevice: send would block."
      );
    }

    throw HardwareError(
      State::UART_SENDING_FAILED, "NetworkCommunicationDevice: send failed with error " + std::to_string(errno)
    );
  }

  if (static_cast<std::size_t>(sent_bytes) != length) {
    throw HardwareError(State::UART_SENDING_FAILED, "Not all bytes were sent over the network.");
  }
  // DEBUG BEGIN    
  // std::cout << "send data: ";
  // for (std::size_t i = 0; i < length; ++i) {
  //   std::cout << std::hex << static_cast<unsigned int>(tx_buffer[i]) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END
}

message::RxMessageDataBuffer NetworkCommunicationDevice::receive()
{
  message::RxMessageDataBuffer rx_buffer;

  if (_socket_fd < 0) {
    throw HardwareError(
      State::UART_RECEIVING_FAILED, "NetworkCommunicationDevice: socket not connected."
    );
  }

  // Read all available data as one message
  ssize_t bytes = recv(
    _socket_fd, 
    static_cast<char*>(static_cast<void*>(rx_buffer.data())),
    rx_buffer.size(), 
    0
  );
  
  if (bytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // No data available (timeout) - return empty buffer
      rx_buffer.clear();
      return rx_buffer;
    }
    throw HardwareError(
      State::UART_RECEIVING_FAILED, 
      "NetworkCommunicationDevice: recv failed with error " + std::to_string(errno)
    );
  }
  
  if (bytes == 0) {
    // Connection closed
    throw HardwareError(
      State::UART_RECEIVING_FAILED, 
      "NetworkCommunicationDevice: connection closed by peer."
    );
  }

  // Resize buffer to actual received bytes
  rx_buffer.resize(static_cast<std::size_t>(bytes));

  // DEBUG BEGIN
  // std::cout << "received data: ";
  // for (const auto& byte : rx_buffer) {
  //   std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END

  return rx_buffer;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
