/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"

#include <array>
#include <edu_robot/rotation_per_minute.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace iotbot {
namespace uart {
namespace message {

struct SetRpm : public MessageFrame<element::Command<UART::COMMAND::SET::RPM>, element::Int16, element::Int16, element::Int16, element::Int16> {
  SetRpm(const Rpm rpm_0, const Rpm rpm_1, const Rpm rpm_2, const Rpm rpm_3)
    : MessageFrame<element::Command<UART::COMMAND::SET::RPM>, element::Int16, element::Int16, element::Int16, element::Int16>(
        static_cast<element::Int16::DataType>(rpm_0 * 100.f + 0.5f),
        static_cast<element::Int16::DataType>(rpm_1 * 100.f + 0.5f),
        static_cast<element::Int16::DataType>(rpm_2 * 100.f + 0.5f),
        static_cast<element::Int16::DataType>(rpm_3 * 100.f + 0.5f)
      )
  { }
};

template <Byte UartCommand>
struct SetLighting : public MessageFrame<element::Command<UartCommand>, element::Uint8, element::Uint8, element::Uint8, /* spacer */ element::Uint8 , element::Uint32> {
  SetLighting(const Color& color)
    : MessageFrame<element::Command<UartCommand>, element::Uint8, element::Uint8, element::Uint8, /* spacer */ element::Uint8 , element::Uint32>(
        color.r, color.g, color.b, 0u, 0u) {
  }
};

template <Byte UartCommand>
struct SetValueF : public MessageFrame<element::Command<UartCommand>, element::Float, /* spacer */ element::Uint32> {
  SetValueF(const element::Float::DataType value)
    : MessageFrame<element::Command<UartCommand>, element::Float, element::Uint32>(value, 0u) {
  }    
};

template <Byte UartCommand>
struct SetValueU : public MessageFrame<element::Command<UartCommand>, element::Uint32, /* spacer */ element::Uint32> {
  SetValueU(const element::Uint32::DataType value)
    : MessageFrame<element::Command<UartCommand>, element::Uint32, element::Uint32>(value, 0u) {
  }
};

struct SetImuRawDataMode : public MessageFrame<element::Command<UART::COMMAND::SET::IMU_RAW_DATA>, element::Uint8, /* spacer */ element::Uint8, element::Int16, element::Uint32> {
  SetImuRawDataMode(const bool enable)
    : MessageFrame<element::Command<UART::COMMAND::SET::IMU_RAW_DATA>, element::Uint8, element::Uint8, element::Int16, element::Uint32>(enable ? 0xff : 0x00, 0u, 0u, 0u) {
  }
};

struct Enable : public MessageFrame<element::Command<UART::COMMAND::ENABLE>, /* spacer */ element::Uint32, element::Uint32> {
  Enable() : MessageFrame<element::Command<UART::COMMAND::ENABLE>, element::Uint32, element::Uint32>(0u, 0u) { }
};

struct Disable : public MessageFrame<element::Command<UART::COMMAND::DISABLE>, /* spacer */ element::Uint32, element::Uint32> {
  Disable() : MessageFrame<element::Command<UART::COMMAND::DISABLE>, element::Uint32, element::Uint32>(0u, 0u) { }
};

struct ShieldResponse : public uart::message::Message<element::Uint8, element::Int16,  element::Int16, element::Int16,  element::Int16, element::Uint32, element::Uint32, element::Int16,
                                                      element::Int16, element::Int16, element::Int16, element::Int16, element::Int16,  element::Int16, element::Uint8>
{
  ShieldResponse(const std::array<Byte, SIZE>& message_candidate)
    : uart::message::Message<element::Uint8, element::Int16,  element::Int16, element::Int16,  element::Int16, element::Uint32, element::Uint32, element::Int16, 
                             element::Int16, element::Int16, element::Int16, element::Int16, element::Int16,  element::Int16, element::Uint8>(message_candidate) { }

  inline constexpr std::uint8_t command() const { return getElementValue<0>(); }
  inline constexpr Rpm rpm0() const { return static_cast<float>(getElementValue<1>()) / 100.0f; }
  inline constexpr Rpm rpm1() const { return static_cast<float>(getElementValue<2>()) / 100.0f; }
  inline constexpr Rpm rpm2() const { return static_cast<float>(getElementValue<3>()) / 100.0f; }
  inline constexpr Rpm rpm3() const { return static_cast<float>(getElementValue<4>()) / 100.0f; }
  inline constexpr float temperature() const { return static_cast<float>(getElementValue<7>()) / 100.0f; }
  inline constexpr float range0() const { return static_cast<float>(getElementValue<9>()) / 1000.0f; }
  inline constexpr float range1() const { return static_cast<float>(getElementValue<10>()) / 1000.0f; }
  inline constexpr float range2() const { return static_cast<float>(getElementValue<11>()) / 1000.0f; }
  inline constexpr float range3() const { return static_cast<float>(getElementValue<12>()) / 1000.0f; }
  inline constexpr float voltage() const { return static_cast<float>(getElementValue<13>()) / 100.0f; }
  inline constexpr float current() const { return static_cast<float>(getElementValue<14>()) / 20.0f; }
};                                                     

} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot