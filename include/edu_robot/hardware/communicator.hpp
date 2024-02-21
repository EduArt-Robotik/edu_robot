/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"
#include "edu_robot/hardware/communication_device.hpp"
#include "edu_robot/hardware/communicator_device_interfaces.hpp"

#include <edu_robot/state.hpp>
#include <edu_robot/hardware_error.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <atomic>
#include <list>
#include <mutex>
#include <functional>
#include <chrono>
#include <cstddef>
#include <thread>
#include <future>
#include <queue>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {

using namespace std::chrono_literals;

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
public:
  using CallbackProcessData = std::function<void(const message::RxMessageDataBuffer&)>;

  RxDataEndPoint(RxDataEndPoint&& rhs) {
    _data_buffer = std::move(rhs._data_buffer);
    _response_search_pattern = std::move(rhs._response_search_pattern);
    _callback_process_data = std::move(rhs._callback_process_data);
    _data_receiver = std::move(rhs._data_receiver);

    _running = true;
    _executer = std::thread(&RxDataEndPoint::processDataJob, this);
  }
  ~RxDataEndPoint()
  {
    _running = false;
    _executer.join();
  }

  /**
   * \brief Creates an data endpoint that processes incoming data from the ethernet gateway.
   * \param callback The callback function that is been called when the message search pattern matches.
   *                 Note: the callback must be threadsafe!
   * \return Return an data endpoint ready to use by the ethernet communicator.
   */
  template <class Message>
  inline static RxDataEndPoint make_data_endpoint(
    const CallbackProcessData& callback, CommunicatorRxDevice* data_receiver)
  {
    const auto search_pattern = Message::makeSearchPattern();
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return RxDataEndPoint(search_pattern_vector, callback, data_receiver);
  }

  inline void call(message::RxMessageDataBuffer&& data) {
    if (_mutex.try_lock() == false) {
      RCLCPP_INFO(rclcpp::get_logger("RxDataEndPoint"), "data receiver is still busy. Drop data.");
      return;
    }

    _data_buffer = std::move(data);
    _mutex.unlock();
  }
  inline const std::vector<message::Byte>& searchPattern() const { return _response_search_pattern; }

protected:
  RxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
    CommunicatorRxDevice* data_receiver)
    : _response_search_pattern(std::move(search_pattern))
    , _callback_process_data(callback_process_data)
    , _data_receiver(data_receiver)
  {
    _executer = std::thread(&RxDataEndPoint::processDataJob, this);
  }

  void processDataJob()
  {
    while (_running) {
      if (_data_receiver == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("RxDataEndPoint"), "data receiver does not longer exist.");
      }

      _mutex.lock();

      if (_data_buffer.empty()) {
        // no data to process --> sleep and try again
        _mutex.unlock();
        std::this_thread::sleep_for(1ms);
        continue;
      }

      std::scoped_lock lock_receiver(_data_receiver->rxDataMutex());
      _callback_process_data(_data_buffer);
      _mutex.unlock();
    }
  }

  message::RxMessageDataBuffer _data_buffer;
  std::atomic_bool _running{true};
  std::thread _executer;
  std::mutex _mutex;

  std::vector<message::Byte> _response_search_pattern;
  std::function<void(const message::RxMessageDataBuffer&)> _callback_process_data;
  CommunicatorRxDevice* _data_receiver;
};

template <typename Request, typename Duration>
inline void wait_for_future(std::future<Request>& future, const Duration& timeout) {
  if (future.wait_for(timeout) == std::future_status::timeout) {
    throw HardwareError(State::SHIELD_REQUEST_TIMEOUT, "Request Timeout!.");
  }
}

template <typename Request>
inline bool is_future_ready(std::future<Request>& future) {
  return future.wait_for(0) == std::future_status::ready;
}

class Communicator
{
  using TaskSendingUart = std::packaged_task<void()>;
  using TaskReceiving = std::packaged_task<message::RxMessageDataBuffer()>;

public:
  Communicator(std::shared_ptr<CommunicationDevice> device, const std::chrono::milliseconds wait_time_sending = 20ms);
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
