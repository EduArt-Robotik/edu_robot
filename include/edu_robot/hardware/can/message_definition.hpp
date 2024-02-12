/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can/message.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can {
namespace message {
namespace sensor {

// EduArt TOF sensor
namespace tof {
using message::MessageFrame;

using StartMeasurement = NoResponseMessageFrame<element::Uint8,  // frame no.
                                                element::Uint8>; // sensor activation bits

struct MeasurementComplete : public MessageFrame<element::Uint8,  // sensor no.
                                                 element::Uint8,  // resolution
                                                 element::Uint8>  // frame no.
{
  inline static constexpr std::size_t zone(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr std::uint8_t resolution(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline static constexpr std::size_t frame(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<3>(rx_buffer);
  } 
};                                                

struct ZoneMeasurement : public MessageFrame<element::Uint8,  // zone no., frame no.
                                             element::Uint24> // distance, sigma
{
  inline static constexpr std::size_t zone(const RxMessageDataBuffer& rx_buffer) {
    return (deserialize<1>(rx_buffer) >> 2) & 0x3f;
  }
  inline static constexpr std::size_t frame(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer) & 0x03;
  }
  inline static float distance(const RxMessageDataBuffer& rx_buffer) {
    return ((*reinterpret_cast<const std::uint32_t*>(&deserialize<2>(rx_buffer)[0]) >> 10) & 0x3fff) / 4.0f / 1000.0f;
  }
  inline static float sigma(const RxMessageDataBuffer& rx_buffer) {
    return ((*reinterpret_cast<const std::uint32_t*>(&deserialize<2>(rx_buffer)[0]) >> 0) & 0x3ff) / 128.0f / 1000.0f;
  }
};

} // end namespace tof
} // end namespace sensor



} // end namespace message
} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
