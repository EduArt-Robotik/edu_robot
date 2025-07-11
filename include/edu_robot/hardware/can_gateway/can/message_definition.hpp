/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/can_gateway/can/message.hpp"
#include "edu_robot/hardware/can_gateway/can/protocol.hpp"

#include <edu_robot/rpm.hpp>

#include <Eigen/Dense>

#include <stdexcept>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
namespace can {
namespace message {
namespace sensor {

// EduArt TOF sensor
namespace tof {
using message::MessageFrame;

// struct ActivationBitfield : public element::Uint8 {
//   inline static constexpr std::array<Byte, size()> serialize(const std::uint8_t sensor_id) {
//     return element::Uint8::serialize(static_cast<std::uint8_t>(1 << (sensor_id - 1)));
//   }
// };

using StartMeasurement = NoResponseMessageFrame<element::Uint8,   // frame no.
                                                element::Uint16>; // sensor activation bits

struct MeasurementComplete : public MessageFrame<element::Uint8>  // frame no.
{
  inline static constexpr std::size_t sensor(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
};
using TriggerDataTransmission = NoResponseMessageFrame<element::Uint16>; // sensor activation bits
struct DataTransmissionComplete : public MessageFrame<element::Uint8, // total points count
                                                      element::Uint8> // frame id
{
  inline static constexpr std::size_t pointCount(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr std::uint8_t frameId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
}; 

struct Measurement : public Message<element::Uint24LE> // distance, sigma
{
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
  inline static float distance(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::distance(selectDataElement(rx_buffer, index));
  }
  inline static float sigma(const RxMessageDataBuffer& rx_buffer, const std::size_t index) {
    return Measurement::sigma(selectDataElement(rx_buffer, index));
  }
  inline static std::size_t elements(const RxMessageDataBuffer& rx_buffer) {
    return (rx_buffer.size() - element::CanAddress::size()) / Measurement::size();
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

namespace imu {

struct MeasurementOrientation : public message::MessageFrame<element::Int16, // orientation w
                                                             element::Int16, // orientation x
                                                             element::Int16, // orientation y
                                                             element::Int16> // orientation z
{
  inline static Eigen::Quaterniond orientation(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Quaterniond(
      static_cast<double>(deserialize<1>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<2>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<3>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<4>(rx_buffer)) / 10000.0
    );
  }
};

struct MeasurementRaw : public message::MessageFrame<element::Uint8, // measurement type indicator
                                                     element::Int16, // measurement 0
                                                     element::Int16, // measurement 1
                                                     element::Int16> // measurement 2
{
  inline static constexpr bool isLinearAcceleration(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer) == 1;
  }
  inline static constexpr bool isAngularVelocity(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer) == 2;
  }
  inline static Eigen::Vector3d linearAcceleration(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Vector3d(
      -static_cast<double>(deserialize<3>(rx_buffer)) / 1000.0f,
      -static_cast<double>(deserialize<4>(rx_buffer)) / 1000.0f,
      static_cast<double>(deserialize<2>(rx_buffer)) / 1000.0f
    );
  }
  inline static Eigen::Vector3d angularVelocity(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Vector3d(
      -static_cast<double>(deserialize<3>(rx_buffer)) / 1000.0f,
      -static_cast<double>(deserialize<4>(rx_buffer)) / 1000.0f,
      static_cast<double>(deserialize<2>(rx_buffer)) / 1000.0f    
    );
  }
};

} // end namespace imu

} // end namespace sensor

// EduArt Motor Controller Boards
namespace motor_controller {

// Message Frame and Message Element Definitions
template <Byte CommandByte, class... Elements>
struct MessageFrame : public message::NoResponseMessageFrame<element::Command<CommandByte>, Elements...>
{
  inline static TxMessageDataBuffer serialize(
    const std::uint32_t can_address, const typename Elements::type&... element_value)
  {
    return message::MessageFrame<element::Command<CommandByte>, Elements...>::serialize(
      can_address, 0, element_value...
    );
  }
};
struct Pwm : public element::Int8 {
  inline static constexpr std::array<Byte, size()> serialize(const float value) {
    if (value < -1.0f || value > 1.0f) {
      throw std::invalid_argument("Pwm: given value is out of range. [-1 .. 1]");
    }
    return element::Int8::serialize(static_cast<std::int8_t>(value * 100.0f));
  }
};
struct Rpm : public element::Int16 {
  inline static constexpr std::array<Byte, size()> serialize(const robot::Rpm value) {
    return element::Int16::serialize(value * 100.0);
  }
  inline static constexpr robot::Rpm deserialize(const Byte data[size()]) {
    return robot::Rpm(static_cast<double>(element::Int16::deserialize(data)) / 100.0);
  }
};

// Message Definitions
using Enable = MessageFrame<PROTOCOL::MOTOR::COMMAND::ENABLE>;
using Disable = MessageFrame<PROTOCOL::MOTOR::COMMAND::DISABLE>;
using SetTimeout = MessageFrame<PROTOCOL::MOTOR::COMMAND::SET_TIMEOUT,
                                element::Uint16>; // timeout in ms
using SetInvertedEncoder = MessageFrame<PROTOCOL::MOTOR::COMMAND::INVERT_ENCODER,
                                        element::Uint8>; // flag inverted encoder
using SetPwm = MessageFrame<PROTOCOL::MOTOR::COMMAND::SET_PWM,
                            Pwm,  // pwm value motor 0
                            Pwm>; // pwm value motor 1
using SetRpm = MessageFrame<PROTOCOL::MOTOR::COMMAND::SET_RPM,
                            Rpm,  // rpm value motor 0
                            Rpm>; // rpm value motor 1
using SetOpenLoop = MessageFrame<PROTOCOL::MOTOR::COMMAND::OPEN_LOOP>;
using SetClosedLoop = MessageFrame<PROTOCOL::MOTOR::COMMAND::CLOSE_LOOP>;
using SetFrequency = MessageFrame<PROTOCOL::MOTOR::COMMAND::FREQUENCY,
                                  element::Uint32>; // motor control frequency in Hz
using SetCtlKp = MessageFrame<PROTOCOL::MOTOR::COMMAND::CTL_KP,
                              element::Float>; // controller kp value
using SetCtlKi = MessageFrame<PROTOCOL::MOTOR::COMMAND::CTL_KI,
                              element::Float>; // controller ki value
using SetCtlKd = MessageFrame<PROTOCOL::MOTOR::COMMAND::CTL_KD,
                              element::Float>; // controller kd value                              
using SetCtlAntiWindUp = MessageFrame<PROTOCOL::MOTOR::COMMAND::CTL_ANTI_WIND_UP,
                                      element::Uint8>; // flag controller anti wind up
using SetCtlInputFilter = MessageFrame<PROTOCOL::MOTOR::COMMAND::CTL_INPUT_FILTER,
                                       element::Float>; // controller input filter weight
using SetGearRatio = MessageFrame<PROTOCOL::MOTOR::COMMAND::GEAR_RATIO,
                                  element::Float>; // gear ratio value for both motors
using SetTicksPerRevision = MessageFrame<PROTOCOL::MOTOR::COMMAND::TICKS_PER_REVISION,
                                         element::Float>; // ticks per revision for encoder
using SetRpmMax = MessageFrame<PROTOCOL::MOTOR::COMMAND::SET_RPM_MAX,
                               element::Float>; // max rpm value for both motors

struct Response : public message::MessageFrame<element::Uint8, // command
                                               Rpm,            // measured rpm motor 0
                                               Rpm,            // measured rpm motor 1
                                               element::Uint8> // enabled flag
{
  inline static constexpr robot::Rpm rpm0(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline static constexpr robot::Rpm rpm1(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<3>(rx_buffer);
  }
  inline static constexpr bool enabled(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<4>(rx_buffer);
  }
};

} // end namespace motor_controller

namespace power_management {

struct Response : public message::MessageFrame<element::Uint8, // measurement type indicator
                                               element::Float, // measurement value
                                               element::Uint8> // enable state
{
  inline static constexpr bool isCurrent(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer) == PROTOCOL::POWER_MANAGEMENT::MEASUREMENT::CURRENT;
  }
  inline static constexpr bool isVoltage(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer) == PROTOCOL::POWER_MANAGEMENT::MEASUREMENT::VOLTAGE;
  }
  inline static constexpr float value(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline static constexpr bool isEnabled(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<3>(rx_buffer);
  }
};

} // end namespace_power_management

namespace can_gateway_shield {

struct Response : public message::MessageFrame<element::Int16, // temperature measurement
                                               element::Int16> // voltage measurement
{
  inline static constexpr float temperature(const RxMessageDataBuffer& rx_buffer) {
    return static_cast<float>(deserialize<1>(rx_buffer)) / 100.0f;
  }
  inline static constexpr float voltage(const RxMessageDataBuffer& rx_buffer) {
    return static_cast<float>(deserialize<2>(rx_buffer));
  }
};

} // end namespace can_gateway_shield

namespace lighting {

struct Sync : public message::NoResponseMessageFrame<element::Command<PROTOCOL::LIGHTING::COMMAND::BEAT>,
                                                     element::Uint8, // new counter value
                                                     element::Uint8> // flag for left side
{
  inline static TxMessageDataBuffer serialize(
    const std::uint32_t can_address, const std::uint8_t new_counter_value, const bool left_side)
  {
    return message::MessageFrame<element::Command<PROTOCOL::LIGHTING::COMMAND::BEAT>, element::Uint8, element::Uint8>::serialize(
      can_address, 0, new_counter_value, left_side ? 1 : 0
    );
  }
};

template <Byte Command>
struct SetLighting : public message::NoResponseMessageFrame<element::Command<Command>, // lighting set command
                                                            element::Uint8, // color r
                                                            element::Uint8, // color g
                                                            element::Uint8> // color b
{
  inline static TxMessageDataBuffer serialize(
    const std::uint32_t can_address, const std::uint8_t red, const std::uint8_t green, const std::uint8_t blue)
  {
    return message::MessageFrame<element::Command<Command>, element::Uint8, element::Uint8, element::Uint8>::serialize(
      can_address, 0, red, green, blue
    );
  }  
};

} // end namespace lighting

} // end namespace message
} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
