/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/communicator.hpp"
#include "edu_robot/hardware/can_gateway/can/message.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
namespace can {

class CanRxDataEndPoint : public hardware::RxDataEndPoint
{
  friend class Communicator;

public:
  /**
   * \brief Creates an data endpoint that processes incoming data from the communicator.
   * \param callback The callback function that is been called when the message search pattern matches.
   *                 Note: the callback must be threadsafe!
   * \return Return an data endpoint ready to use by the ethernet communicator.
   */
  template <class Message>
  inline static std::shared_ptr<RxDataEndPoint> make_data_endpoint(
    std::shared_ptr<Executer> executer, const std::uint32_t can_id, const CallbackProcessData& callback,
    const std::uint8_t buffer_size = 5)
  {
    const auto search_pattern = Message::makeSearchPattern(can_id);
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return std::shared_ptr<CanRxDataEndPoint>(
      new CanRxDataEndPoint(search_pattern_vector, callback, executer, buffer_size)
    );
  }

  inline static std::shared_ptr<RxDataEndPoint> make_data_endpoint(
    std::shared_ptr<Executer> executer, const std::uint32_t can_id, const CallbackProcessData& callback,
    const std::uint8_t buffer_size = 5)
  {
    const auto search_pattern = message::MessageFrame<>::makeSearchPattern(can_id);
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return std::shared_ptr<CanRxDataEndPoint>(
      new CanRxDataEndPoint(search_pattern_vector, callback, executer, buffer_size)
    );
  }

private:
  CanRxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
    std::shared_ptr<Executer> executer,
    const std::uint8_t buffer_size)
  : hardware::RxDataEndPoint(search_pattern, callback_process_data, executer, buffer_size)
  { }
};  

} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
