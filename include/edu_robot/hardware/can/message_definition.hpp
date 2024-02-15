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

struct ActivationBitfield : public element::Uint16 {
  inline static constexpr std::array<Byte, size()> serialize(const std::uint8_t sensor_id) {
    return element::Uint16::serialize((1 << (sensor_id - 1)));
  }
};

using StartMeasurement = NoResponseMessageFrame<element::Uint8,      // frame no.
                                                ActivationBitfield>; // sensor activation bits

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

struct Measurement : public Message<element::Uint24, // distance, sigma
                                    element::Uint8>  // zone no., frame no.
{
  inline static constexpr std::size_t zone(
    const std::array<RxMessageDataBuffer::value_type, Measurement::size()>& rx_buffer)
  {
    return (deserialize<1>(rx_buffer) >> 2) & 0x3f;
  }
  inline static constexpr std::size_t frame(
    const std::array<RxMessageDataBuffer::value_type, Measurement::size()>& rx_buffer)
  {
    return deserialize<1>(rx_buffer) & 0x03;
  }
  inline static float distance(const std::array<RxMessageDataBuffer::value_type, Measurement::size()>& rx_buffer)
  {
    return ((*reinterpret_cast<const std::uint32_t*>(&deserialize<0>(rx_buffer)[0]) >> 10) & 0x3fff) / 4.0f / 1000.0f;
  }
  inline static float sigma(const std::array<RxMessageDataBuffer::value_type, Measurement::size()>& rx_buffer)
  {
    return ((*reinterpret_cast<const std::uint32_t*>(&deserialize<0>(rx_buffer)[0]) >> 0) & 0x3ff) / 128.0f / 1000.0f;
  } 
};

struct ZoneMeasurement : public MessageFrame<> // use empty message frame as base
{
  inline static std::size_t zone(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::zone(selectDataElement(rx_buffer, index));
  }
  inline static std::size_t frame(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::frame(selectDataElement(rx_buffer, index));
  }
  inline static float distance(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::distance(selectDataElement(rx_buffer, index));
  }
  inline static float sigma(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::sigma(selectDataElement(rx_buffer, index));
  }

private:
  static std::array<RxMessageDataBuffer::value_type, Measurement::size()> selectDataElement(
    const RxMessageDataBuffer& rx_buffer, const std::size_t index)
  {
    const auto begin = rx_buffer.begin() + element::CanAddress::size() + index * Measurement::size();
    const auto end = begin + Measurement::size();
    std::array<RxMessageDataBuffer::value_type, Measurement::size()> measurement;
    std::copy(begin, end, measurement.begin());

    return measurement;
  }
};

} // end namespace tof
} // end namespace sensor



} // end namespace message
} // end namespace can
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
