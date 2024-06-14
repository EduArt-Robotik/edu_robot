/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"

#include <atomic>
#include <functional>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

namespace eduart {
namespace robot {
namespace hardware {

class CommunicatorRxNode;

class RxDataEndPoint
{
public:
  using CallbackProcessData = std::function<void(const message::RxMessageDataBuffer&)>;

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
  inline static std::shared_ptr<RxDataEndPoint> make_data_endpoint(
    CommunicatorRxNode* data_receiver, const CallbackProcessData& callback, const std::uint8_t buffer_size = 1)
  {
    const auto search_pattern = Message::makeSearchPattern();
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return std::shared_ptr<RxDataEndPoint>(
      new RxDataEndPoint(search_pattern_vector, callback, data_receiver, buffer_size)
    );
  }

  void call(const message::RxMessageDataBuffer& data);
  void deactivate();
  inline const std::vector<message::Byte>& searchPattern() const { return _response_search_pattern; }

protected:
  RxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
    CommunicatorRxNode* data_receiver,
    const std::uint8_t buffer_size);

  void processDataJob();

  std::vector<std::shared_ptr<message::RxMessageDataBuffer>> _input_data_buffer;
  std::queue<std::shared_ptr<message::RxMessageDataBuffer>> _output_data_buffer;
  std::atomic_bool _running{true};
  std::thread _executer;
  std::mutex _mutex;

  std::vector<message::Byte> _response_search_pattern;
  std::function<void(const message::RxMessageDataBuffer&)> _callback_process_data;
  CommunicatorRxNode* _data_receiver;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
