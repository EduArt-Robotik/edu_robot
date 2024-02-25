#include "edu_robot/hardware/can_gateway/can_communication_device.hpp"

#include <edu_robot/hardware_error.hpp>

#include <rclcpp/logging.hpp>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstring>
#include <unistd.h>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

CanCommunicationDevice::CanCommunicationDevice(char const* const device_name, const CanType can_type)
  : _can_type(can_type)
{
  // Creating Socket CAN Instance
  _socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (_socket_fd < 0) {
    throw HardwareError(State::CAN_SOCKET_ERROR, "Error occurred during opening CAN socket.");
  }

  // Enable CAN FD if needed
  if (_can_type == CanType::CAN_FD) {
    int option = 1;

    if (setsockopt(
      _socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &option, sizeof(option)) < 0) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Can't set can device to CAN FD.");
    }
  }

  // Binding CAN Interface
  ifreq ifr;
  sockaddr_can addr;

  strcpy(ifr.ifr_name, device_name);

  if (::ioctl(_socket_fd, SIOCGIFINDEX, &ifr) < 0) {
    throw HardwareError(State::CAN_SOCKET_ERROR, "Can't get CAN socket index.");
  }

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(_socket_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
    throw HardwareError(State::CAN_SOCKET_ERROR, "Can't bind CAN socket.");
  }
}

CanCommunicationDevice::~CanCommunicationDevice()
{
  ::close(_socket_fd);
}

void CanCommunicationDevice::send(message::Byte const *const tx_buffer, const std::size_t length)
{
  if (length < 4 || length > 12) {
    throw std::invalid_argument("CanCommunicationDevice::send(): given length must be in range 4 .. 12.");
  }

  // Debugging
  std::stringstream debug_out;
  debug_out << "sending: " << std::hex << static_cast<int>(*reinterpret_cast<const std::uint32_t*>(&tx_buffer[0])) << "#";

  for (std::size_t i = 4; i < length; ++i) {
    debug_out << static_cast<int>(tx_buffer[i]) << " ";
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("CanCommunicationDevice"), debug_out.str());

  // Sending Given Data
  if (_can_type == CanType::CAN) {
    can_frame frame;

    frame.can_id = static_cast<int>(*reinterpret_cast<const std::uint32_t*>(&tx_buffer[0])); // first 4 bytes are the address
    frame.len = length - 4;

    for (std::size_t i = 4; i < length; ++i) {
      frame.data[i - 4] = tx_buffer[i];
    }

    if (::write(_socket_fd, &frame, sizeof(can_frame)) != sizeof(can_frame)) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Can't write data to interface.");
    }
  }
  else if (_can_type == CanType::CAN_FD) {
    canfd_frame frame;

    frame.can_id = static_cast<int>(*reinterpret_cast<const std::uint32_t*>(&tx_buffer[0])); // first 4 bytes are the address
    frame.len = length - 4;
    frame.flags = 0;

    for (std::size_t i = 4; i < length; ++i) {
      frame.data[i - 4] = tx_buffer[i];
    }

    if (::send(_socket_fd, &frame, sizeof(canfd_frame), 0) != sizeof(canfd_frame)) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Can't write data to interface.");
    }
  }
  // else: do nothing
}

message::RxMessageDataBuffer CanCommunicationDevice::receive()
{
  // Receiving Data from Interface
  message::RxMessageDataBuffer rx_buffer;

  if (_can_type == CanType::CAN) {
    can_frame frame;

    const auto bytes = ::recv(_socket_fd, &frame, sizeof(can_frame), 0);

    if (bytes < 0) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Can't read data from interface.");
    }
    else if (bytes != sizeof(can_frame)) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Received can frame has wrong size.");
    }
    // else
    rx_buffer.resize(frame.len + sizeof(can_frame::can_id)); // need memory for data + address
    std::memcpy(&rx_buffer[0], &frame.can_id, sizeof(can_frame::can_id));
    std::memcpy(&rx_buffer[sizeof(can_frame::can_id)], frame.data, frame.len);
  }
  else if (_can_type == CanType::CAN_FD) {
    canfd_frame frame;

    const auto bytes = ::recv(_socket_fd, &frame, sizeof(canfd_frame), 0);

    if (bytes < 0) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Can't read data from interface.");
    }
    else if (bytes != sizeof(canfd_frame)) {
      throw HardwareError(State::CAN_SOCKET_ERROR, "Received canfd frame has wrong size.");
    }
    // else
    rx_buffer.resize(frame.len + sizeof(canfd_frame::can_id)); // need memory for data + address
    std::memcpy(&rx_buffer[0], &frame.can_id, sizeof(canfd_frame::can_id));
    std::memcpy(&rx_buffer[sizeof(canfd_frame::can_id)], frame.data, frame.len);    
  }

  // Debugging
  std::stringstream debug_out;
  debug_out << "received: " << std::hex;

  for (const auto byte : rx_buffer) {
    debug_out << static_cast<int>(byte) << " ";
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("CanCommunicationDevice"), debug_out.str());

  return rx_buffer;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
