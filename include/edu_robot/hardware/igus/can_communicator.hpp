/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/message.hpp"
#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <algorithm>
#include <atomic>
#include <list>
#include <mutex>
#include <functional>
#include <chrono>
#include <thread>
#include <future>
#include <queue>
#include <vector>

namespace eduart {
namespace robot {
namespace igus {

class Request
{
  friend class CanCommunicator;

  enum class State {
    CREATED,
    SENT,
    RECEIVED,
    ACCEPTED,
    REJECTED,
  };

public:
  template <class Message, class... Arguments>
  inline static Request make_request(const can::message::Byte can_address, Arguments&&... args) {
    return Request(Message{}, can_address, std::forward<Arguments>(args)...);
  }
  Request(Request&&) = default;

  inline const can::message::RxMessageDataBuffer& response() const { return _response_message; }

private:
  template <can::message::Byte CommandByte, class... Elements>
  Request(const can::message::MessageFrame<CommandByte, Elements...>, const can::message::Byte can_address, const typename Elements::type&... element_value) {
    _request_message = can::message::MessageFrame<CommandByte, Elements...>::serialize(can_address, element_value...);
    const auto search_pattern = can::message::MessageFrame<CommandByte, Elements...>::makeSearchPattern(can_address);

    _response_search_pattern.resize(search_pattern.size());
    std::copy(search_pattern.begin(), search_pattern.end(), _response_search_pattern.begin());
  }

  State _state = State::CREATED;
  can::message::TxMessageDataBuffer _request_message;
  can::message::RxMessageDataBuffer _response_message;
  std::chrono::time_point<std::chrono::system_clock> _was_sent_at;
  std::chrono::time_point<std::chrono::system_clock> _was_received_at;  
  std::vector<can::message::Byte> _response_search_pattern;

  static std::uint8_t _sequence_number;
};

class RxDataEndPoint
{
  friend class CanCommunicator;

public:
  using CallbackProcessData = std::function<void(const can::message::RxMessageDataBuffer&)>;

  RxDataEndPoint(RxDataEndPoint&&) = default;

  /**
   * \brief Creates an data endpoint that processes incoming data from the ethernet gateway.
   * \param callback The callback function that is been called when the message search pattern matches.
   *                 Note: the callback must be threadsafe!
   * \return Return an data endpoint ready to use by the ethernet communicator.
   */
  template <class Message>
  inline static RxDataEndPoint make_data_endpoint(const CallbackProcessData& callback) {
    const auto search_pattern = Message::makeSearchPattern();
    std::vector<can::message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return RxDataEndPoint(search_pattern, callback);
  }

private:
  template <class Message>
  RxDataEndPoint(
    std::vector<can::message::Byte>& search_pattern,
    std::function<void(const can::message::RxMessageDataBuffer&)>& callback_process_data)
    : _response_search_pattern(std::move(search_pattern))
    , _callback_process_data(callback_process_data)
  { }

  std::vector<can::message::Byte> _response_search_pattern;
  std::function<void(const can::message::RxMessageDataBuffer&)> _callback_process_data;
};

template <typename Duration>
inline void wait_for_future(std::future<Request>& future, const Duration& timeout) {
  if (future.wait_for(timeout) == std::future_status::timeout) {
    throw HardwareError(State::SHIELD_REQUEST_TIMEOUT, "TCP Request Timeout!.");
  }
}

class CanCommunicator
{
  using TaskSendingUart = std::packaged_task<void()>;
  using TaskReceiving = std::packaged_task<can::message::RxMessageDataBuffer()>;

public:
  CanCommunicator(char const* const can_device);
  ~CanCommunicator();

  std::future<Request> sendRequest(Request request);
  void registerRxDataEndpoint(RxDataEndPoint&& endpoint);
  can::message::RxMessageDataBuffer getRxBuffer();

private:
  void processSending(const std::chrono::milliseconds wait_time_after_sending);
  void processReceiving();
  void processing();
  void sendingData(can::message::Byte const *const tx_buffer, const std::size_t length);
  can::message::RxMessageDataBuffer receivingData();

  // Ethernet Interface
  int _socket_fd = -1;


  // Handling Thread
  std::thread _handling_thread;
  std::mutex _mutex_data_input;
  std::mutex _mutex_rx_data_endpoint;
  std::atomic_bool _is_running;
  std::atomic_bool _new_incoming_requests;
  std::queue<std::pair<Request, std::promise<Request>>> _incoming_requests;
  std::queue<std::pair<std::pair<Request, std::promise<Request>>, std::future<void>>> _is_being_send;
  std::list<std::pair<Request, std::promise<Request>>> _open_request;
  std::vector<RxDataEndPoint> _rx_data_endpoint;

  // Sending Thread
  std::chrono::milliseconds _wait_time_after_sending;
  std::queue<TaskSendingUart> _sending_in_progress;
  std::mutex _mutex_sending_data;
  std::thread _tcp_sending_thread;

  // Reading Thread
  static constexpr std::size_t _max_rx_buffer_queue_size = 100;
  std::mutex _mutex_receiving_data;
  std::mutex _mutex_received_data_copy;
  std::thread _tcp_receiving_thread;
  std::queue<can::message::RxMessageDataBuffer> _rx_buffer_queue;
  can::message::RxMessageDataBuffer _rx_buffer_copy;
  std::atomic_bool _new_received_data;

  std::list<Request> _open_response_tasks;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
