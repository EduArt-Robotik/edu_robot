/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <array>

namespace eduart {
namespace robot {
namespace iotbot {
namespace uart {
namespace message {

using Byte = std::uint8_t;

struct UART {
  struct BUFFER {
    static constexpr std::size_t TX_SIZE = 11u;
    static constexpr std::size_t RX_SIZE = 32u;
  
    static constexpr Byte START_BYTE = 0xff;
    static constexpr Byte END_BYTE   = 0xee;
  };
  struct COMMAND {
    static constexpr Byte ENABLE  = 0x01;
    static constexpr Byte DISABLE = 0x02;
    struct SET {
      static constexpr Byte UART_TIMEOUT = 0x0c;
      static constexpr Byte IMU_RAW_DATA = 0x0a;
      static constexpr Byte RPM = 0x11;
      static constexpr Byte CONTROL_FREQUENCY = 0x12;      
      static constexpr Byte KP = 0x20;
      static constexpr Byte KI = 0x21;
      static constexpr Byte KD = 0x22;
      static constexpr Byte SET_POINT_LOW_PASS = 0x24;
      static constexpr Byte ENCODER_LOW_PASS = 0x25;
      static constexpr Byte GEAR_RATIO = 0x30;
      static constexpr Byte TICKS_PER_REV = 0x31;
    };

    struct LIGHTING {
      static constexpr Byte OFF = 0x42;
      static constexpr Byte DIM = 0x43;
      static constexpr Byte HIGH_BEAM = 0x44;
      struct FLASH {
        static constexpr Byte ALL = 0x45;
        static constexpr Byte LEFT = 0x46;
        static constexpr Byte RIGHT = 0x47;
      };
      static constexpr Byte PULSATION = 0x48;
      static constexpr Byte ROTATION = 0x49;
      static constexpr Byte RUNNING = 0x4A;
    };
  };
};

namespace element {
namespace impl {

template <typename ElementDataType>
struct Element {
  static constexpr std::size_t SIZE = sizeof(ElementDataType);
  using DataType = ElementDataType;

  // It is a default serialization. On demand customization should take place in derived classes.
  static std::array<Byte, SIZE> serialize(const DataType value)
  {
    union {
      const DataType input;
      const Byte bytes[SIZE];
    } const serializer{ value };

    std::array<Byte, SIZE> serialized_bytes;
    std::memcpy(serialized_bytes.data(), serializer.bytes, serialized_bytes.size());

    return serialized_bytes;
  }
};

template <Byte Value>
struct ConstByte : public Element<Byte> {
  static constexpr Byte VALUE = Value;
  static std::array<Byte, SIZE> serialize() { return {{ VALUE }}; }
};

} // end namespace impl

using StartByte = impl::ConstByte<UART::BUFFER::START_BYTE>;
using StopByte = impl::ConstByte<UART::BUFFER::END_BYTE>;

template <Byte UartCommand>
struct Command : public impl::ConstByte<UartCommand> { };

struct Float : public impl::Element<float> { };
struct Int16 : public impl::Element<std::int16_t> { };
struct Uint32 : public impl::Element<std::uint32_t> { };

} // end namespace element

namespace impl {

template <class ...Elements>
struct Message {
  static constexpr std::size_t SIZE = (Elements::SIZE + ...);

  const std::array<Byte, SIZE>& data() const { return _buffer; }

protected:
  std::array<Byte, SIZE> _buffer;
};

template <class CommandByte, class ...Elements>
struct TxMessageFrame : public Message<element::StartByte, CommandByte, Elements..., element::StopByte>
{
private:
  using Message<element::StartByte, CommandByte, Elements..., element::StopByte>::_buffer;

public:
  using Message<element::StartByte, CommandByte, Elements..., element::StopByte>::SIZE;

  TxMessageFrame(const typename Elements::DataType... args) {
    std::size_t current_index = 0u;

    _buffer[current_index] = element::StartByte::serialize()[0];
    current_index += element::StartByte::SIZE;
    _buffer[current_index] = CommandByte::serialize()[0];
    current_index += CommandByte::SIZE;

    ([&]{
      const auto message_bytes = Elements::serialize(args);
      std::memcpy(
        &Message<element::StartByte, CommandByte, Elements..., element::StopByte>::_buffer[current_index],
        message_bytes.data(),
        Elements::SIZE
      );
      current_index += Elements::SIZE;
    }(),...);

    _buffer[current_index] = element::StopByte::serialize()[0];
    current_index += element::StopByte::SIZE;
  }
};

} // end namespace impl

using SetRpm = impl::TxMessageFrame<element::Command<UART::COMMAND::SET::RPM>,
                                    element::Int16, element::Int16, element::Int16, element::Int16>;

} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
