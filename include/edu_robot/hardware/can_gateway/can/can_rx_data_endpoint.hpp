/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/communicator.hpp"

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
  inline static RxDataEndPoint make_data_endpoint(
    const std::uint32_t can_id, const CallbackProcessData& callback, CommunicatorRxDevice* data_receiver)
  {
    const auto search_pattern = Message::makeSearchPattern(can_id);
    std::vector<message::Byte> search_pattern_vector(search_pattern.begin(), search_pattern.end());
    return CanRxDataEndPoint(search_pattern_vector, callback, data_receiver);
  }

private:
  CanRxDataEndPoint(
    std::vector<message::Byte>& search_pattern,
    const std::function<void(const message::RxMessageDataBuffer&)>& callback_process_data,
    CommunicatorRxDevice* data_receiver)
  : hardware::RxDataEndPoint(search_pattern, callback_process_data, data_receiver)
  { }
};  

} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
