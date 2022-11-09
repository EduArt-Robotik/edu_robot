#include "edu_robot/ethernet_gateway/ethernet_communicator.hpp"
#include "edu_robot/ethernet_gateway/tcp/message.hpp"
#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"
#include "edu_robot/state.hpp"

#include <edu_robot/hardware_error.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <sys/socket.h>
#include <thread>
#include <utility>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace eduart {
namespace robot {
namespace ethernet {

using namespace std::chrono_literals;

EthernetCommunicator::EthernetCommunicator(char const* const ip_address, const std::uint16_t port)
  : _is_running(true)
  , _new_incoming_requests(false)
  , _wait_time_after_sending(1ms) // \todo check if wait time is still needed.
  , _new_received_data(false)
{
  _socket_address.sin_addr.s_addr = inet_addr(ip_address);
	_socket_address.sin_family = AF_INET;
	_socket_address.sin_port = htons( port );
  _socket_fd = ::socket(AF_INET , SOCK_STREAM , 0);

  if (_socket_fd < -1) {
    throw HardwareError(State::TCP_SOCKET_ERROR, "Error occurred during opening TCP socket.");
  }
  if (::connect(_socket_fd, (struct sockaddr*)&_socket_address, sizeof(_socket_address)) < 0) {
    ::close(_socket_fd);
    throw HardwareError(State::TCP_SOCKET_ERROR, "Error occurred during opening TCP socket.");
  }

  _tcp_sending_thread = std::thread([this]{
    processSending(_wait_time_after_sending);
  });
  _tcp_receiving_thread = std::thread([this]{
    processReceiving();
  });
  _handling_thread = std::thread([this]{
    processing();
  });
}

EthernetCommunicator::~EthernetCommunicator()
{
  std::cout << "Shuting down TCP communicator." << std::endl;

  _is_running = false;
  _handling_thread.join();
  _tcp_sending_thread.join();
  _tcp_receiving_thread.join();  
  ::close(_socket_fd);
}

std::future<Request> EthernetCommunicator::sendRequest(Request request)
{
  std::promise<Request> feedback;
  auto future = feedback.get_future();
  {
    std::lock_guard lock(_mutex_data_input);
    _incoming_requests.emplace(std::move(request), std::move(feedback));
    _new_incoming_requests = true;
  }

  return future;
}

tcp::message::RxMessageDataBuffer EthernetCommunicator::getRxBuffer()
{
  // Make received data available for polling.
  std::lock_guard guard(_mutex_received_data_copy);
  return _rx_buffer_copy;
}

template <typename Left, typename Right>
static bool is_same(const Left& lhs, const Right& rhs) {
  if (lhs.size() > rhs.size()) {
    return false;;
  }

  bool is_same = true;
  
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    is_same &= lhs[i] == rhs[i];
  }

  return is_same;
}

void EthernetCommunicator::processing()
{
  while (_is_running) {
    auto start = std::chrono::system_clock::now();

    // Process New Input Requests
    if (_new_incoming_requests) {
      // Get next tx message that should be sent.
      std::scoped_lock lock(_mutex_sending_data, _mutex_data_input);
      auto request = std::move(_incoming_requests.front());
      _incoming_requests.pop();
      // Add tx message to sending queue.
      tcp::message::Byte const *const tx_buffer = request.first._request_message.data();
      const std::size_t length = request.first._request_message.size();
      TaskSendingUart task([this, tx_buffer, length]{
        sendingData(tx_buffer, length);
      });
      auto future = task.get_future();
      _sending_in_progress.emplace(std::move(task));
      _is_being_send.emplace(std::move(request), std::move(future));
      _new_incoming_requests = false;
    }

    // Process Being Sent Messages
    if (!_is_being_send.empty() && _is_being_send.front().second.wait_for(0ms) == std::future_status::ready) {
      // Sending is complete.
      auto request = std::move(_is_being_send.front().first);
      auto future = std::move(_is_being_send.front().second);
      _is_being_send.pop();

      try {
        // If get doesn't throw an exception --> sending was ok.
        future.get();
        // Request was successfully sent to the shield.
        request.first._state = Request::State::SENT;
        _open_request.emplace_back(std::move(request));
      }
      catch (...) {
        // Forward exception to original requester.
        request.second.set_exception(std::current_exception());
      }
    }

    // Handle Received Tcp Data
    if (_new_received_data == true) {
      // Reading data thread is waiting.
      try {
        tcp::message::RxMessageDataBuffer rx_buffer;
        {
          std::unique_lock lock(_mutex_receiving_data);
          rx_buffer = std::move(_rx_buffer_queue.front());
          _rx_buffer_queue.pop();

          if (_rx_buffer_queue.size() == 0) {
            _new_received_data = false;
          }
        }        
        {
          // Make received data available for polling.
          std::lock_guard guard(_mutex_received_data_copy);
          _rx_buffer_copy = rx_buffer;
        }

        for (auto it =_open_request.begin(); it != _open_request.end();) {
          const auto& search_pattern = it->first._response_search_pattern;

          if (is_same(search_pattern, rx_buffer)) {
            // Found Corresponding Request
            auto request = std::move(*it);
            it = _open_request.erase(it);

            request.first._state = Request::State::RECEIVED;
            request.first._response_message = rx_buffer;
            request.second.set_value(std::move(request.first));
            break;
          }
          // else
          ++it;
        }
      }
      catch (std::exception& ex) {
        // \todo this class needs an logger!
        std::cout << "Error occurred during reading TCP interface: what() = " << ex.what() << std::endl;
      }
    }

    // Make a period of 1ms
    // \todo fix lines below. Its a bit nasty.
    if (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start) < 1ms) {
      const auto diff = 1ms - std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start);
      std::this_thread::sleep_for(diff);      
    }
  }
}

void EthernetCommunicator::sendingData(tcp::message::Byte const *const tx_buffer, const std::size_t length)
{
  // \todo think about it! If there is only one line here remove the sending method.
  ::send(_socket_fd, tx_buffer, length, 0);
}

tcp::message::RxMessageDataBuffer EthernetCommunicator::receivingData()
{
  tcp::message::RxMessageDataBuffer rx_buffer;

  // Allocate memory and store received message in.
  rx_buffer.reserve(tcp::message::MAX_MESSAGE_SIZE);
  const int received_bytes = ::recv(_socket_fd, rx_buffer.data(), rx_buffer.size(), MSG_CMSG_CLOEXEC);

  if (received_bytes < 0) {
    return { };
  }

  // Set correct size to rx buffer.
  rx_buffer.resize(received_bytes);

  return rx_buffer;
}

void EthernetCommunicator::processSending(const std::chrono::milliseconds wait_time_after_sending)
{
  std::chrono::time_point<std::chrono::system_clock> last_sending = std::chrono::system_clock::now();

  while (_is_running) {
    // Wait a minimum time until next message can be sent.
    auto now = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sending) < wait_time_after_sending) {
      const auto diff = wait_time_after_sending - std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sending);
      std::this_thread::sleep_for(diff);
      now = std::chrono::system_clock::now();
    }       

    // Check if data is available for sending.
    bool sending_data_available;
    {
      std::lock_guard guard(_mutex_sending_data);
      sending_data_available = _sending_in_progress.empty() == false;
    }

    if (sending_data_available == false) {
      std::this_thread::sleep_for(1ms);
      continue;      
    }

    // Execute Task if Available
    TaskSendingUart task;
    {
      std::lock_guard guard(_mutex_sending_data);
      task = std::move(_sending_in_progress.front());
      _sending_in_progress.pop();
    }

    task();
    last_sending = std::chrono::system_clock::now();
  }
}

void EthernetCommunicator::processReceiving()
{
  while (_is_running) {
    int count;
    ioctl(_socket_fd, FIONREAD, &count);

    if (count <= 0) {
      std::this_thread::sleep_for(1ms);
      continue;
    }

    try {
      tcp::message::RxMessageDataBuffer rx_buffer;
      rx_buffer = receivingData();

      {
        std::unique_lock lock(_mutex_receiving_data);
        _rx_buffer_queue.emplace(rx_buffer);
        _new_received_data = true;
      }
    }
    catch (std::exception& ex) {
      std::cout << "error occurred during receiving data: what() = " << ex.what() << std::endl;
      std::cout << "drop message" << std::endl;
      continue;
    }
  }
}

} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
