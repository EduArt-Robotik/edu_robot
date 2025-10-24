/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/iot_shield/uart/message.hpp"

#include <Eigen/Dense>

#include <edu_robot/rpm.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {
namespace uart {
namespace message {

using uart::message::MessageFrame;

using SetRpm = MessageFrame<element::Command<UART::COMMAND::SET::RPM>,
                            element::Int16,  // RPM set point motor 1
                            element::Int16,  // RPM set point motor 2
                            element::Int16,  // RPM set point motor 3
                            element::Int16>; // RPM set point motor 4

template <Byte UartCommand>
using SetLighting = MessageFrame<element::Command<UartCommand>,
                                 element::Uint8,   // r value
                                 element::Uint8,   // g value
                                 element::Uint8,   // b value
                                 element::Uint8,   // spacer
                                 element::Uint32>; // spacer


template <Byte UartCommand>
using SetValueF = MessageFrame<element::Command<UartCommand>,
                               element::Float,   // float value
                               element::Uint32>; // spacer

template <Byte UartCommand>
using SetValueU = MessageFrame<element::Command<UartCommand>,
                               element::Uint32,  // unsigned int value
                               element::Uint32>; // spacer

template <Byte UartCommand>
using SetFlag = MessageFrame<element::Command<UartCommand>,
                             element::Uint8,   // flag
                             element::Uint8,   // spacer
                             element::Int16,   // spacer
                             element::Uint32>; // spacer

using SetImuRawDataMode = MessageFrame<element::Command<UART::COMMAND::SET::IMU_RAW_DATA>,
                                       element::Uint8,   // enable flag
                                       element::Uint8,   // spacer
                                       element::Int16,   // spacer
                                       element::Uint32>; // spacer

using Enable = MessageFrame<element::Command<UART::COMMAND::ENABLE>,
                            element::Uint32,  // spacer
                            element::Uint32>; // spacer

using Disable = MessageFrame<element::Command<UART::COMMAND::DISABLE>,
                             element::Uint32,  // spacer
                             element::Uint32>; // spacer

struct ShieldResponse : public uart::message::Message<element::Uint8, element::Int16,  element::Int16, element::Int16,  element::Int16, element::Int16, element::Int16, element::Int16, element::Int16, element::Int16,
                                                      element::Int16, element::Int16, element::Int16, element::Int16, element::Int16,  element::Int16, element::Uint8>
{
  inline static constexpr std::uint8_t command(const RxMessageDataBuffer& rx_buffer) { return deserialize<0>(rx_buffer); }
  inline static constexpr Rpm rpm0(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<1>(rx_buffer)) / 100.0f; }
  inline static constexpr Rpm rpm1(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<2>(rx_buffer)) / 100.0f; }
  inline static constexpr Rpm rpm2(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<3>(rx_buffer)) / 100.0f; }
  inline static constexpr Rpm rpm3(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<4>(rx_buffer)) / 100.0f; }
  inline static Eigen::Quaterniond imuOrientation(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Quaterniond(
      static_cast<double>(deserialize<5>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<6>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<7>(rx_buffer)) / 10000.0,
      static_cast<double>(deserialize<8>(rx_buffer)) / 10000.0
    );
  }
  inline static Eigen::Vector3d linearAcceleration(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Vector3d(
      // Convert from [mg/s] to [m/s^2].
      // Data are received in following order:
      // index 5 = -z, index 6 = y, index 7 = -x
       static_cast<double>(deserialize<7>(rx_buffer)) / 10000.0 * 9.81,
      -static_cast<double>(deserialize<6>(rx_buffer)) / 10000.0 * 9.81,
       static_cast<double>(deserialize<5>(rx_buffer)) / 10000.0 * 9.81
    );
  }
  inline static Eigen::Vector3d angularVelocity(const RxMessageDataBuffer& rx_buffer) {
    return Eigen::Vector3d(
      // Convert from [mdeg/s] to [rad/s].
      // Data are received in following order:
      // index 8 = -z, index 9 = y, index 10 = x
      (-static_cast<double>(deserialize< 9>(rx_buffer)) / 100.0) * (M_PI / 180.0),
      ( static_cast<double>(deserialize<10>(rx_buffer)) / 100.0) * (M_PI / 180.0),
      ( static_cast<double>(deserialize< 8>(rx_buffer)) / 100.0) * (M_PI / 180.0)
    );
  }
  inline static constexpr float temperature(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<9>(rx_buffer)) / 100.0f; }
  inline static constexpr float range0(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<11>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range1(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<12>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range2(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<13>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range3(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<14>(rx_buffer)) / 1000.0f; }
  inline static constexpr float voltage(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<15>(rx_buffer)) / 100.0f; }
  inline static constexpr float current(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<16>(rx_buffer)) / 20.0f; }
  inline constexpr static auto makeSearchPattern() {
    return std::array<Byte, 0>();;
  }
};                                                     

} // end namespace message
} // end namespace uart
} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
