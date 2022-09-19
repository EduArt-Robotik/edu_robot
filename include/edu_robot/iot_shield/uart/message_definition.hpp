/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"

#include <edu_robot/rotation_per_minute.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace iotbot {
namespace uart {
namespace message {

struct SetRpm : public MessageFrame<element::Command<UART::COMMAND::SET::RPM>,
                                    element::Int16, element::Int16, element::Int16, element::Int16>
{
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

// element::Uint8,  element::Int16,  element::Int16, element::Int16,  element::Int16,
//                                        element::Uint32, element::Uint32, element::Int16, element::Uint32, element::Uint32,
//                                        element::Int16,  element::Int16,  element::Uint8

} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot