/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"
#include "edu_robot/iot_shield/uart/message_definition.hpp"
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

    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }

  State _state = State::CREATED;
  uart::message::TxMessageDataBuffer _request_message;
  uart::message::RxMessageDataBuffer _response_message;
  std::chrono::time_point<std::chrono::system_clock> _was_sent_at;
  std::chrono::time_point<std::chrono::system_clock> _was_received_at;  
  std::vector<uart::message::Byte> _response_search_pattern;
  std::promise<ShieldRequest> _response; //> will be set by communicator
};

class IotShieldCommunicator
{
  using TaskSendingUart = std::packaged_task<void()>;
  using TaskReceiving = std::packaged_task<uart::message::RxMessageDataBuffer()>;

public:
  IotShieldCommunicator(char const* const device_name);
  ~IotShieldCommunicator();

  void registerProcessReceivedBytes(std::function<void(const uart::message::RxMessageDataBuffer&)> callback);
  std::future<ShieldRequest> sendRequest(ShieldRequest request);

private:
  void processSending(const std::chrono::milliseconds wait_time_after_sending);
  void processReceiving(TaskReceiving task);
  void processing();
  void sendingData(const uart::message::TxMessageDataBuffer& tx_buffer);
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
  std::queue<ShieldRequest> _incoming_requests;
  std::queue<std::pair<ShieldRequest, std::future<void>>> _is_being_send;
  std::list<ShieldRequest> _open_request;
  std::future<uart::message::RxMessageDataBuffer> _future_received_data;

  // Sending Thread
  std::chrono::milliseconds _wait_time_after_sending;

  std::queue<TaskSendingUart> _sending_in_progress;
  std::mutex _mutex_sending_data;
  std::thread _uart_sending_thread;

  // Reading Thread
  std::mutex _mutex_receiving_data;
  std::thread _uart_receiving_thread;

  std::list<ShieldRequest> _open_response_tasks;
};

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
