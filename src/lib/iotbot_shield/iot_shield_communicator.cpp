#include "edu_robot/iot_shield/iot_shield_communicator.hpp"
#include "edu_robot/hardware_error.hpp"
#include "edu_robot/iot_shield/iot_shield_device.hpp"
#include "edu_robot/iot_shield/uart/message.hpp"

#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

namespace eduart {
namespace robot {
namespace iotbot {

using namespace std::chrono_literals;

IotShieldCommunicator::IotShieldCommunicator(char const* const device_name)
  : _is_running(false)
  , _wait_time_after_sending(8ms)
{
  // \todo check for better logging instance.
#if _WITH_MRAA
  std::cout << "open uart device \"" << device_name << "\"" << std::endl;
  _uart = std::make_unique<mraa::Uart>(device_name);

  if (_uart->setBaudRate(115200) != mraa::SUCCESS) {
     std::cerr << "Error setting parity on UART" << std::endl;
  }

  if (_uart->setMode(8, mraa::UART_PARITY_NONE, 1) != mraa::SUCCESS) {
     std::cerr << "Error setting parity on UART" << std::endl;
  }

  if (_uart->setFlowcontrol(false, false) != mraa::SUCCESS) {
     std::cerr << "Error setting flow control UART" << std::endl;
  }
   
  _uart->flush();
#else
  (void)device_name;
  std::cerr << "UART interface not available. MRAA is missing!" << std::endl;
#endif

  _uart_sending_thread = std::thread([this]{
    processSending(_wait_time_after_sending);
  });
  _uart_receiving_thread = std::thread([this]{
    TaskReceiving task([this]{ return receivingData(); });
    _future_received_data = task.get_future();
    processReceiving(std::move(task));
  });
  _handling_thread = std::thread([this]{
    processing();
  });
}

IotShieldCommunicator::~IotShieldCommunicator()
{
#if _WITH_MRAA
  _uart->flush();
#endif
  _is_running = false;
  _handling_thread.join();
  _uart_sending_thread.join();
  _uart_receiving_thread.join();  
}

void IotShieldCommunicator::registerProcessReceivedBytes(
  std::function<void(const uart::message::RxMessageDataBuffer&)> callback)
{
  _process_received_bytes = callback;
}

std::future<ShieldRequest> IotShieldCommunicator::sendRequest(ShieldRequest request)
{
  auto future = request._response.get_future();
  {
    std::lock_guard lock(_mutex_data_input);
    _incoming_requests.emplace(std::move(request));
    _new_incoming_requests = true;
  }

  return future;
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

void IotShieldCommunicator::processing()
{
  while (_is_running) {
    // Process New Input Requests
    if (_new_incoming_requests) {
      std::scoped_lock lock(_mutex_sending_data, _mutex_data_input);
      ShieldRequest request = std::move(_incoming_requests.front());
      _incoming_requests.pop();
      TaskSendingUart task([this, &request]{
        sendingData(request._request_message);
      });
      auto future = task.get_future();
      _sending_in_progress.emplace(std::move(task));
      _is_being_send.emplace(std::move(request), std::move(future));
    }

    // Process Being Sent Messages
    if (!_is_being_send.empty() && _is_being_send.front().second.wait_for(0ms) == std::future_status::ready) {
      // Sending is complete.
      ShieldRequest&& request = std::move(_is_being_send.front().first);
      auto future = std::move(_is_being_send.front().second);
      _is_being_send.pop();

      try {
        // If get doesn't throw an exception --> sending was ok.
        future.get();
        // Request was successfully sent to the shield.
        request._state = ShieldRequest::State::SENT;
        _open_request.emplace_back(std::move(request));
      }
      catch (...) {
        // Forward exception to original requester.
        request._response.set_exception(std::current_exception());
      }
    }

    // Handle Received Uart Data
    if (_future_received_data.wait_for(0ms) == std::future_status::ready) {
      try {
        const auto& rx_buffer = _future_received_data.get();

        for (auto it =_open_request.begin(); it != _open_request.end();) {
          const auto& search_pattern = it->_response_search_pattern;

          if (is_same(search_pattern, rx_buffer)) {
            // Found Corresponding Request
            auto request = std::move(*it);
            it = _open_request.erase(it);

            request._state = ShieldRequest::State::RECEIVED;
            request._response_message = rx_buffer;
            request._response.set_value(std::move(request));
             // \todo is it okay to delete promise here? Because the request is deleted at this point.
            break;
          }
        }
      }
      catch (std::exception& ex) {
        // \todo this class needs an logger!
        std::cout << "Error occurred during reading UART interface: what() = " << ex.what() << std::endl;
      }
    }
  }
}

void IotShieldCommunicator::sendingData(const uart::message::TxMessageDataBuffer& tx_buffer)
{
#if _WITH_MRAA
  const std::size_t sent_bytes = _uart->write((char*)tx_buffer.data(), tx_buffer.size());
    
  std::cout << "send data: ";
  for (const auto& byte : tx_buffer) {
    std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  }
  std::cout << std::endl;

  if (sent_bytes != tx_buffer.size()) {
    throw HardwareError(State::UART_SENDING_FAILED, "Less byte sent as expected.");
  }
#else
(void)tx_buffer;
#endif
}

uart::message::RxMessageDataBuffer IotShieldCommunicator::receivingData()
{
  uart::message::RxMessageDataBuffer rx_buffer;
  std::size_t received_bytes = 0;

#if _WITH_MRAA
  while (received_bytes + 1 < rx_buffer.size() && _uart->dataAvailable(100)) {
    received_bytes += _uart->read(rx_buffer.data() + received_bytes, 1);
  }
#endif

  if (received_bytes != rx_buffer.size()) {
    throw HardwareError(State::UART_RECEIVING_FAILED, "Received bytes do not fit to rx buffer.");
  }

  // DEBUG BEGIN
  std::cout << "received data: ";
  for (const auto& byte : rx_buffer) {
    std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  }
  std::cout << std::endl;
  // DEBUG END

  return rx_buffer;
}

void IotShieldCommunicator::processSending(const std::chrono::milliseconds wait_time_after_sending)
{
  std::chrono::time_point<std::chrono::system_clock> last_sending = std::chrono::system_clock::now();

  while (_is_running) {
    // Wait a minimum time until next message can be sent.
    // \todo Try to find a non blocking/sleeping solution.
    auto now = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sending) < wait_time_after_sending) {
      const auto diff = wait_time_after_sending - std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sending);
      std::this_thread::sleep_for(diff);
      now = std::chrono::system_clock::now();
    }       

    // Execute Task if Available
    TaskSendingUart task;
    {
      std::lock_guard guard(_mutex_sending_data);

      if (_sending_in_progress.empty()) {
        continue;
      }

      task = std::move(_sending_in_progress.front());
      _sending_in_progress.pop();
    }

    task();
    last_sending = std::chrono::system_clock::now();
  }
}

void IotShieldCommunicator::processReceiving(TaskReceiving task)
{
  while (_is_running) {
#if _WITH_MRAA
    if (_uart->dataAvailable(1000) == false) {
      continue;
    }
#else
  std::this_thread::sleep_for(1000ms);
#endif
    task();
  }
}

// void IotShieldCommunicator::processUartCommunication()
// {
//   _processing_is_running = true;

//   while (_processing_is_running) {
//     std::unique_lock look(_mutex_data_input);
//     _cv_data_input.wait(look, [this]{ return this->_tx_input_data_ready; });

//     // Wait a minimum time until next message can be sent.
//     // \todo Try to find a non blocking/sleeping solution.
//     auto now = std::chrono::system_clock::now();

//     while (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_sending) < _wait_time_after_sending) {
//       const auto diff = _wait_time_after_sending - std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_sending);
//       std::this_thread::sleep_for(diff);
//       now = std::chrono::system_clock::now();
//     }

#if _WITH_MRAA
    _uart->write((char*)_tx_buffer.data(), _tx_buffer.size());
    std::cout << "send data: ";
    for (const auto& byte : _tx_buffer) {
      std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
    }
    std::cout << std::endl;

    _last_sending = now;


    uart::message::RxMessageDataBuffer rx_buffer;
    std::size_t received_bytes = 0;

    while (received_bytes + 1 < rx_buffer.size() && _uart->dataAvailable(100)) {
      received_bytes += _uart->read(rx_buffer.data() + received_bytes, 1);
    }

    if (received_bytes != rx_buffer.size()) {
      std::cout << __PRETTY_FUNCTION__ << "\n" << "received bytes do not fit to rx buffer" << std::endl;
      continue;
    }

    std::cout << "received data: ";
    for (const auto& byte : rx_buffer) {
      std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
    }
    std::cout << std::endl;
    _process_received_bytes(rx_buffer);
#endif
  // }
// }

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
