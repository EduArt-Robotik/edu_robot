/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {

struct UART {
  struct BUFFER {
    static constexpr std::size_t TX_SIZE = 11u;
    static constexpr std::size_t RX_SIZE = 32u;
  
    static constexpr std::uint8_t START_BYTE = 0xff;
    static constexpr std::uint8_t END_BYTE   = 0xee;
  };
  struct COMMAND {
    static constexpr std::uint8_t ENABLE  = 0x01;
    static constexpr std::uint8_t DISABLE = 0x02;

    struct LIGHTING {
      static constexpr std::uint8_t OFF = 0x42;
      static constexpr std::uint8_t DIM = 0x43;
      static constexpr std::uint8_t HIGH_BEAM = 0x44;
      struct FLASH {
        static constexpr std::uint8_t ALL = 0x45;
        static constexpr std::uint8_t LEFT = 0x46;
        static constexpr std::uint8_t RIGHT = 0x47;
      };
      static constexpr std::uint8_t PULSATION = 0x48;
      static constexpr std::uint8_t ROTATION = 0x49;
      static constexpr std::uint8_t RUNNING = 0x4A;
    };
  };
};

template<std::size_t LOWER_BYTE, std::size_t UPPER_BYTE>
inline std::int16_t rxBufferToInt16(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer)
{
  static_assert(LOWER_BYTE < UART::BUFFER::RX_SIZE && UPPER_BYTE < UART::BUFFER::RX_SIZE,
                "Rx Buffer is too small --> given index is out of range.");

  return (static_cast<std::int16_t>(buffer[UPPER_BYTE]) << 8) + static_cast<std::int16_t>(buffer[LOWER_BYTE]);
}

inline float rxBufferToTemperature(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer)
{
  return static_cast<float>(rxBufferToInt16<17, 18>(buffer)) / 100.0f;
}

inline float rxBufferToVoltage(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer)
{
  return static_cast<float>(rxBufferToInt16<29, 30>(buffer)) / 100.0f;
}

inline float rxBufferToCurrent(const std::array<std::uint8_t, UART::BUFFER::RX_SIZE>& buffer)
{
  constexpr std::size_t idx = 31u;
  static_assert(idx < UART::BUFFER::RX_SIZE, "Rx Buffer is too small --> given index is out of range.");

  return static_cast<float>(buffer[idx]) / 20.0f;
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
