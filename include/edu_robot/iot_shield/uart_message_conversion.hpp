/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <cstring>

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
    struct SET {
      static constexpr std::uint8_t UART_TIMEOUT = 0x0c;
      static constexpr std::uint8_t IMU_RAW_DATA = 0x0a;
      static constexpr std::uint8_t RPM = 0x11;
      static constexpr std::uint8_t CONTROL_FREQUENCY = 0x12;      
      static constexpr std::uint8_t KP = 0x20;
      static constexpr std::uint8_t KI = 0x21;
      static constexpr std::uint8_t KD = 0x22;
      static constexpr std::uint8_t SET_POINT_LOW_PASS = 0x24;
      static constexpr std::uint8_t ENCODER_LOW_PASS = 0x25;
      static constexpr std::uint8_t GEAR_RATIO = 0x30;
      static constexpr std::uint8_t TICKS_PER_REV = 0x31;
    };

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

  return (static_cast<std::int16_t>(buffer[UPPER_BYTE]) << 8) | static_cast<std::int16_t>(buffer[LOWER_BYTE]);
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

template<std::size_t LOWER_BYTE, std::size_t UPPER_BYTE>
inline void int16ToTxBuffer(const std::int16_t value, std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& buffer)
{
  static_assert(LOWER_BYTE < UART::BUFFER::TX_SIZE && UPPER_BYTE < UART::BUFFER::TX_SIZE,
                "Tx Buffer is too small --> given index is out of range.");

  buffer[UPPER_BYTE] = static_cast<std::uint8_t>((value & 0xff00) >> 8);
  buffer[LOWER_BYTE] = static_cast<std::uint8_t>((value & 0x00ff) >> 0);               
}

template<std::size_t LOWER_BYTE, std::size_t UPPER_BYTE>
inline void uint32ToTxBuffer(const std::uint32_t value, std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& buffer)
{
  static_assert(LOWER_BYTE < UART::BUFFER::TX_SIZE && UPPER_BYTE < UART::BUFFER::TX_SIZE,
                "Tx Buffer is too small --> given index is out of range.");
  static_assert(std::abs(static_cast<int>(UPPER_BYTE) - static_cast<int>(LOWER_BYTE)) == 3,
                "Distance of given byte indices is not correct. Four bytes are needed for serializing a uint32.");

  constexpr int idx_step = (static_cast<int>(UPPER_BYTE) - static_cast<int>(LOWER_BYTE)) > 0 ? 1 : -1; 
  union SerializerUint32 {
    std::uint32_t number;
    std::uint8_t bytes[sizeof(std::uint32_t)];
  };

  SerializerUint32 serializer{ value };

  buffer[UPPER_BYTE]            = serializer.bytes[3];
  buffer[UPPER_BYTE - idx_step] = serializer.bytes[2];
  buffer[LOWER_BYTE + idx_step] = serializer.bytes[1];
  buffer[LOWER_BYTE]            = serializer.bytes[0];  
}

template<std::size_t LOWER_BYTE, std::size_t UPPER_BYTE>
inline void floatToTxBuffer(const float value, std::array<std::uint8_t, UART::BUFFER::TX_SIZE>& buffer)
{
  static_assert(LOWER_BYTE < UART::BUFFER::TX_SIZE && UPPER_BYTE < UART::BUFFER::TX_SIZE,
                "Tx Buffer is too small --> given index is out of range.");
  static_assert(std::abs(static_cast<int>(UPPER_BYTE) - static_cast<int>(LOWER_BYTE)) == 3,
                "Distance of given byte indices is not correct. Four bytes are needed for serializing a float.");

  constexpr int idx_step = (static_cast<int>(UPPER_BYTE) - static_cast<int>(LOWER_BYTE)) > 0 ? 1 : -1;
  union ConverterUint32Float {
    std::uint32_t uint_number;
    float real_number;
    std::uint8_t bytes[sizeof(float)];
  };

  ConverterUint32Float converter;
  converter.real_number = value;
  buffer[UPPER_BYTE]            = converter.bytes[3];
  buffer[UPPER_BYTE - idx_step] = converter.bytes[2];
  buffer[LOWER_BYTE + idx_step] = converter.bytes[1];
  buffer[LOWER_BYTE]            = converter.bytes[0];
}

} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
