/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"

#include <edu_robot/executer.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>

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

  }

  /**
   * \brief Creates an data endpoint that processes incoming data from the ethernet gateway.
   * \param executer Executes this data endpoint.
   * \param callback The callback function that is been called when the message search pattern matches.
   *                 Note: the callback must be threadsafe!
   * \return Return an data endpoint ready to use by the ethernet communicator.
   */
  template <class Message>
  inline static std::shared_ptr<RxDataEndPoint> make_data_endpoint(
    std::shared_ptr<Executer> executer, const CallbackProcessData& callback, const std::uint8_t buffer_size = 1)
  {
    const auto search_pattern = Message::makeSearchPattern();
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return std::shared_ptr<RxDataEndPoint>(
      new RxDataEndPoint(search_pattern_vector, callback, executer, buffer_size)
    );
  }

  void call(const message::RxMessageDataBuffer& data);
  void deactivate();
  inline const std::vector<message::Byte>& searchPattern() const { return _response_search_pattern; }

protected:
  RxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
    std::shared_ptr<Executer> executer,
    const std::uint8_t buffer_size);

  void processDataJob();

  std::vector<std::unique_ptr<message::RxMessageDataBuffer>> _input_data_buffer;
  std::queue<std::unique_ptr<message::RxMessageDataBuffer>> _output_data_buffer;
  std::atomic_bool _running{true};
  std::shared_ptr<Executer> _executer;
  std::mutex _mutex;

  std::vector<message::Byte> _response_search_pattern;
  std::function<void(const message::RxMessageDataBuffer&)> _callback_process_data;
};

} // end namespace igus
} // end namespace eduart
} // end namespace robot
