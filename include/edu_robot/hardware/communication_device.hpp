/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/message_buffer.hpp"

namespace eduart {
namespace robot {
namespace hardware {

/**
 * \brief This interface class is used for communication with the hardware. It is used by the Communicator class. It needs
 *        to be implemented for each concrete hardware like CAN oder ethernet.
 */
class CommunicationDevice
{
protected:
  CommunicationDevice() = default;

public:
  virtual ~CommunicationDevice() = default;
  
  virtual void send(message::Byte const *const tx_buffer, const std::size_t length) = 0;
  virtual message::RxMessageDataBuffer receive() = 0;
};

} // end namespace hardware
} // end namespace eduart
} // end namespace robot
