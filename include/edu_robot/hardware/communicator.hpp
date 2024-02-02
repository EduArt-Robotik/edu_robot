/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"
#include "edu_robot/hardware/communication_device.hpp"

#include <cstddef>
#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

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
namespace hardware {

class Request
{
  friend class Communicator;

  enum class State {
    CREATED,
    SENT,
    RECEIVED,
    ACCEPTED,
    REJECTED,
  };

public:
  Request(Request&&) = default;
  virtual ~Request() = default;

  inline const message::RxMessageDataBuffer& response() const { return _response_message; }

protected:
  Request() = default;

  State _state = State::CREATED;
  message::TxMessageDataBuffer _request_message;
  message::RxMessageDataBuffer _response_message;
  std::chrono::time_point<std::chrono::system_clock> _was_sent_at;
  std::chrono::time_point<std::chrono::system_clock> _was_received_at;  
  std::vector<message::Byte> _response_search_pattern;
};

class RxDataEndPoint
{
  friend class Communicator;

public:
  using CallbackProcessData = std::function<void(const message::RxMessageDataBuffer&)>;

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
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return RxDataEndPoint(search_pattern, callback);
  }

private:
  template <class Message>
  RxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data)
    : _response_search_pattern(std::move(search_pattern))
    , _callback_process_data(callback_process_data)
  { }

  std::vector<message::Byte> _response_search_pattern;
  std::function<void(const message::RxMessageDataBuffer&)> _callback_process_data;
};

template <typename Request, typename Duration>
inline void wait_for_future(std::future<Request>& future, const Duration& timeout) {
  if (future.wait_for(timeout) == std::future_status::timeout) {
    throw HardwareError(State::SHIELD_REQUEST_TIMEOUT, "Request Timeout!.");
  }
}

class Communicator
{
  using TaskSendingUart = std::packaged_task<void()>;
  using TaskReceiving = std::packaged_task<message::RxMessageDataBuffer()>;

public:
  Communicator(std::shared_ptr<CommunicationDevice> device);
  ~Communicator();

  std::future<Request> sendRequest(Request request);
  void registerRxDataEndpoint(RxDataEndPoint&& endpoint);
  message::RxMessageDataBuffer getRxBuffer();

private:
  void processSending(const std::chrono::milliseconds wait_time_after_sending);
  void processReceiving();
  void processing();

  // Communication Interface
  std::shared_ptr<CommunicationDevice> _device;

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
  std::queue<message::RxMessageDataBuffer> _rx_buffer_queue;
  message::RxMessageDataBuffer _rx_buffer_copy;
  std::atomic_bool _new_received_data;

  std::list<Request> _open_response_tasks;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
