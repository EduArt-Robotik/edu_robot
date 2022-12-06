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
  : _is_running(true)
  , _new_incoming_requests(false)
  , _wait_time_after_sending(8ms)
  , _new_received_data(false)
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
  while (_uart->dataAvailable(1) == true) { char buffer; _uart->read(&buffer, 1); }
#else
  (void)device_name;
  std::cerr << "UART interface not available. MRAA is missing!" << std::endl;
#endif

  _uart_sending_thread = std::thread([this]{
    processSending(_wait_time_after_sending);
  });
  _uart_receiving_thread = std::thread([this]{
    processReceiving();
  });
  _handling_thread = std::thread([this]{
    processing();
  });
}

IotShieldCommunicator::~IotShieldCommunicator()
{
  std::cout << "Shuting down shield communicator." << std::endl;
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
  std::promise<ShieldRequest> feedback;
  auto future = feedback.get_future();
  {
    std::lock_guard lock(_mutex_data_input);
    _incoming_requests.emplace(std::move(request), std::move(feedback));
    _new_incoming_requests = true;
  }

  return future;
}

uart::message::RxMessageDataBuffer IotShieldCommunicator::getRxBuffer()
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

void IotShieldCommunicator::processing()
{
  while (_is_running) {
    auto start = std::chrono::system_clock::now();

    // Process New Input Requests
    if (_new_incoming_requests) {
      std::scoped_lock lock(_mutex_sending_data, _mutex_data_input);
      auto request = std::move(_incoming_requests.front());
      _incoming_requests.pop();
      // Add tx message to sending queue. The data pointer must not be changed as long the data will be sent. 
      uart::message::Byte const *const tx_buffer = request.first._request_message.data();
      constexpr std::size_t length = request.first._request_message.size();
      TaskSendingUart task([this, tx_buffer]{
        sendingData(tx_buffer, length); // length is constexpr --> no need to captured it.
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
        request.first._state = ShieldRequest::State::SENT;
        _open_request.emplace_back(std::move(request));
      }
      catch (...) {
        // Forward exception to original requester.
        request.second.set_exception(std::current_exception());
      }
    }

    // Handle Received Uart Data
    if (_new_received_data == true) {
      // Reading data thread is waiting.
      try {
        uart::message::RxMessageDataBuffer rx_buffer;
        {
          std::unique_lock lock(_mutex_receiving_data);
          rx_buffer = _rx_buffer_queue.front();
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

            request.first._state = ShieldRequest::State::RECEIVED;
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
        std::cout << "Error occurred during reading UART interface: what() = " << ex.what() << std::endl;
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

void IotShieldCommunicator::sendingData(uart::message::Byte const *const tx_buffer, const std::size_t length)
{
#if _WITH_MRAA
  const char* data = static_cast<const char*>(static_cast<const void*>(tx_buffer));
  const std::size_t sent_bytes = _uart->write(data, length);
  
  // DEBUG BEGIN    
  // std::cout << "send data: ";
  // for (const auto& byte : tx_buffer) {
  //   std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END

  if (sent_bytes != length) {
    throw HardwareError(State::UART_SENDING_FAILED, "Less byte sent as expected.");
  }

#else
  (void)tx_buffer;
  (void)length;
#endif
}

uart::message::RxMessageDataBuffer IotShieldCommunicator::receivingData()
{
  uart::message::RxMessageDataBuffer rx_buffer;
  std::size_t received_bytes = 0;

#if _WITH_MRAA
  while (received_bytes < rx_buffer.size() && _uart->dataAvailable(100)) {
    received_bytes += _uart->read(
      static_cast<char*>(static_cast<void*>(rx_buffer.data())) + received_bytes, 1
    );
  }
#endif

  // DEBUG BEGIN
  // std::cout << "received data: ";
  // for (const auto& byte : rx_buffer) {
  //   std::cout << std::hex << static_cast<unsigned int>(byte) << " ";
  // }
  // std::cout << std::endl;
  // DEBUG END

  if (received_bytes != rx_buffer.size()) {
    throw HardwareError(State::UART_RECEIVING_FAILED, "Received bytes do not fit to rx buffer.");
  }

  return rx_buffer;
}

void IotShieldCommunicator::processSending(const std::chrono::milliseconds wait_time_after_sending)
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

void IotShieldCommunicator::processReceiving()
{
  while (_is_running) {
#if _WITH_MRAA
    if (_uart->dataAvailable(1000) == false) {
      continue;
    }
#else
  std::this_thread::sleep_for(1000ms);
#endif

    try {
      uart::message::RxMessageDataBuffer rx_buffer;
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
    // {
    //   std::unique_lock lock(_mutex_receiving_data);
    //   _new_received_data = true;
    //   _cv_receiving_data.wait(lock);
    // }
  }
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
