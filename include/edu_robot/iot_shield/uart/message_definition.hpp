/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/iot_shield/uart/message.hpp"

#include <Eigen/Dense>
#include <array>
#include <edu_robot/rotation_per_minute.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace iotbot {
namespace uart {
namespace message {

using SetRpm = MessageFrame<element::Command<UART::COMMAND::SET::RPM>, element::Int16, element::Int16, element::Int16, element::Int16>;
//   SetRpm(const Rpm rpm_0, const Rpm rpm_1, const Rpm rpm_2, const Rpm rpm_3)
//     : MessageFrame<element::Command<UART::COMMAND::SET::RPM>, element::Int16, element::Int16, element::Int16, element::Int16>(
//         static_cast<element::Int16::DataType>(rpm_0 * 100.f + 0.5f),
//         static_cast<element::Int16::DataType>(rpm_1 * 100.f + 0.5f),
//         static_cast<element::Int16::DataType>(rpm_2 * 100.f + 0.5f),
//         static_cast<element::Int16::DataType>(rpm_3 * 100.f + 0.5f)
//       )
//   { }
// };

template <Byte UartCommand>
using SetLighting = MessageFrame<element::Command<UartCommand>, element::Uint8, element::Uint8, element::Uint8, /* spacer */ element::Uint8 , element::Uint32>;
//   SetLighting(const Color& color)
//     : MessageFrame<element::Command<UartCommand>, element::Uint8, element::Uint8, element::Uint8, /* spacer */ element::Uint8 , element::Uint32>(
//         color.r, color.g, color.b, 0u, 0u) {
//   }
// };

template <Byte UartCommand>
using SetValueF = MessageFrame<element::Command<UartCommand>, element::Float, /* spacer */ element::Uint32>;
//   SetValueF(const element::Float::DataType value)
//     : MessageFrame<element::Command<UartCommand>, element::Float, element::Uint32>(value, 0u) {
//   }    
// };

template <Byte UartCommand>
using SetValueU = MessageFrame<element::Command<UartCommand>, element::Uint32, /* spacer */ element::Uint32>;
//   SetValueU(const element::Uint32::DataType value)
//     : MessageFrame<element::Command<UartCommand>, element::Uint32, element::Uint32>(value, 0u) {
//   }
// };

using SetImuRawDataMode = MessageFrame<element::Command<UART::COMMAND::SET::IMU_RAW_DATA>, element::Uint8, /* spacer */ element::Uint8, element::Int16, element::Uint32>;
//   SetImuRawDataMode(const bool enable)
//     : MessageFrame<element::Command<UART::COMMAND::SET::IMU_RAW_DATA>, element::Uint8, element::Uint8, element::Int16, element::Uint32>(enable ? 0xff : 0x00, 0u, 0u, 0u) {
//   }
// };

using Enable = MessageFrame<element::Command<UART::COMMAND::ENABLE>, /* spacer */ element::Uint32, element::Uint32>;
//   Enable() : MessageFrame<element::Command<UART::COMMAND::ENABLE>, element::Uint32, element::Uint32>(0u, 0u) { }
// };

using Disable = MessageFrame<element::Command<UART::COMMAND::DISABLE>, /* spacer */ element::Uint32, element::Uint32>;
//   Disable() : MessageFrame<element::Command<UART::COMMAND::DISABLE>, element::Uint32, element::Uint32>(0u, 0u) { }
// };

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
      static_cast<double>(deserialize<5>(rx_buffer)) / 1000.0,
      static_cast<double>(deserialize<6>(rx_buffer)) / 1000.0,
      static_cast<double>(deserialize<7>(rx_buffer)) / 1000.0,
      static_cast<double>(deserialize<8>(rx_buffer)) / 1000.0
    );
  }
  inline static constexpr float temperature(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<9>(rx_buffer)) / 100.0f; }
  inline static constexpr float range0(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<11>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range1(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<12>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range2(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<13>(rx_buffer)) / 1000.0f; }
  inline static constexpr float range3(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<14>(rx_buffer)) / 1000.0f; }
  inline static constexpr float voltage(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<15>(rx_buffer)) / 100.0f; }
  inline static constexpr float current(const RxMessageDataBuffer& rx_buffer) { return static_cast<float>(deserialize<16>(rx_buffer)) / 20.0f; }
};                                                     

} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot