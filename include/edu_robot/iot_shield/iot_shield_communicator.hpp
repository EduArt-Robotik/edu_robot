/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <algorithm>
#include <atomic>
#include <list>
#include <mutex>
#include <utility>
#include <vector>

#if _WITH_MRAA
#include <mraa/common.hpp>
#include <mraa/uart.hpp>
#endif

#include <functional>
#include <memory>
#include <array>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <future>
#include <queue>

namespace eduart {
namespace robot {
namespace iotbot {

class IotShieldDevice;

class ShieldRequest
{
  friend class IotShieldCommunicator;

  enum class State {
    CREATED,
    SENT,
    RECEIVED,
    ACCEPTED,
    REJECTED,
  };

public:
  template <class Message, class... Arguments>
  inline static ShieldRequest make_request(Arguments&&... args) {
    return ShieldRequest(Message{}, std::forward<Arguments>(args)...);
  }
  ShieldRequest(ShieldRequest&&) = default;

private:
  template <class CommandByte, class... Elements>
  ShieldRequest(const uart::message::MessageFrame<CommandByte, Elements...>, const typename Elements::type&... element_value) {
    _request_message = uart::message::MessageFrame<CommandByte, Elements...>::serialize(element_value...);
    const auto search_pattern = uart::message::MessageFrame<CommandByte, Elements...>::makeSearchPattern();

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }

  State _state = State::CREATED;
  uart::message::TxMessageDataBuffer _request_message;
  uart::message::RxMessageDataBuffer _response_message;
  std::chrono::time_point<std::chrono::system_clock> _was_sent_at;
  std::chrono::time_point<std::chrono::system_clock> _was_received_at;  
  std::vector<uart::message::Byte> _response_search_pattern;
};

template <typename Duration>
inline void wait_for_future(std::future<ShieldRequest>& future, const Duration& timeout) {
  if (future.wait_for(timeout) == std::future_status::timeout) {
    throw HardwareError(State::SHIELD_REQUEST_TIMEOUT, "UART Request Timeout! Cancel set RPM to motor controller.");
  }
}

class IotShieldCommunicator
{
  using TaskSendingUart = std::packaged_task<void()>;
  using TaskReceiving = std::packaged_task<uart::message::RxMessageDataBuffer()>;

public:
  IotShieldCommunicator(char const* const device_name);
  ~IotShieldCommunicator();

  void registerProcessReceivedBytes(std::function<void(const uart::message::RxMessageDataBuffer&)> callback);
  std::future<ShieldRequest> sendRequest(ShieldRequest request);
  uart::message::RxMessageDataBuffer getRxBuffer();

private:
  void processSending(const std::chrono::milliseconds wait_time_after_sending);
  void processReceiving();
  void processing();
  void sendingData(uart::message::Byte const *const tx_buffer, const std::size_t length);
  uart::message::RxMessageDataBuffer receivingData();

  std::function<void(const uart::message::RxMessageDataBuffer&)> _process_received_bytes;

#if _WITH_MRAA
  std::unique_ptr<mraa::Uart> _uart;
#endif

  // Handling Thread
  std::thread _handling_thread;
  std::mutex _mutex_data_input;
  std::atomic_bool _is_running;
  std::atomic_bool _new_incoming_requests;
  std::queue<std::pair<ShieldRequest, std::promise<ShieldRequest>>> _incoming_requests;
  std::queue<std::pair<std::pair<ShieldRequest, std::promise<ShieldRequest>>, std::future<void>>> _is_being_send;
  std::list<std::pair<ShieldRequest, std::promise<ShieldRequest>>> _open_request;

  // Sending Thread
  std::chrono::milliseconds _wait_time_after_sending;

  std::queue<TaskSendingUart> _sending_in_progress;
  std::mutex _mutex_sending_data;
  std::thread _uart_sending_thread;

  // Reading Thread
  std::mutex _mutex_receiving_data;
  std::mutex _mutex_received_data_copy;
  std::thread _uart_receiving_thread;
  std::queue<uart::message::RxMessageDataBuffer> _rx_buffer_queue;
  uart::message::RxMessageDataBuffer _rx_buffer_copy;
  std::atomic_bool _new_received_data;

  std::list<ShieldRequest> _open_response_tasks;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
