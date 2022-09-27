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
#include <tuple>
#include <type_traits>
#include <utility>

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
  static constexpr std::array<Byte, SIZE> serialize(const DataType value)
  {
    union {
      const Byte* bytes;
      const DataType* input;
    } const serializer{ reinterpret_cast<const Byte*>(&value) };

    std::array<Byte, SIZE> serialized_bytes{ 0 };
    std::memcpy(serialized_bytes.data(), serializer.bytes, serialized_bytes.size());

    return serialized_bytes;
  }
  static constexpr DataType deserialize(const Byte data[SIZE])
  {
    union {
      DataType value;
      Byte bytes[SIZE];
    } serializer{ };
    std::memcpy(serializer.bytes, data, SIZE);

    return serializer.value;
  }
  inline static constexpr bool isElementValid(const Byte[SIZE]) { return true; }
};

template <typename ElementDataType, ElementDataType Value>
struct ConstElement : public Element<ElementDataType> {
  static constexpr ElementDataType VALUE = Value;
  static constexpr std::size_t SIZE = sizeof(ElementDataType);
  static constexpr std::array<Byte, SIZE> serialize(const ElementDataType) { return {{ VALUE }}; }
  static constexpr bool isElementValid(const Byte data[SIZE]) {
    return VALUE == Element<ElementDataType>::deserialize(data);
  }
};

/**
 * \brief This class constructs a UART message based on the given message elements. Methods
 *        to (de-)serialize and to check received message candidate are provided.
 *
 * \todo make a static version for received message checking --> no instantiation necessary.
 */
template <std::size_t Index, std::size_t ByteIndex, class... Elements>
struct Message;

template <std::size_t Index, std::size_t ByteIndex>
struct Message<Index, ByteIndex>{
protected:
  inline constexpr bool isElementValid() const { return true; }

  std::array<Byte, ByteIndex> _buffer;
};

template <std::size_t Index, std::size_t ByteIndex, class HeadElement, class... TailElements>
struct Message<Index, ByteIndex, HeadElement, TailElements...>
  : public Message<Index + 1, ByteIndex + HeadElement::SIZE, TailElements...>
{
protected:
  static constexpr std::size_t BYTE_INDEX = ByteIndex;
  using Message<Index + 1, ByteIndex + HeadElement::SIZE, TailElements...>::_buffer;

  Message() = default;

  template <class HeadArgument, class... TailArguments>
  constexpr Message(const HeadArgument head_arg, const TailArguments... tail_args)
    : Message<Index + 1, ByteIndex + HeadElement::SIZE, TailElements...>(tail_args...)
  {
    const auto serialized_bytes = serialize(head_arg);
    std::memcpy(&_buffer[BYTE_INDEX], serialized_bytes.data(), serialized_bytes.size());
  }
  inline constexpr bool isElementValid() const {
    return HeadElement::isElementValid(&_buffer[BYTE_INDEX])
           &&
           Message<Index + 1, ByteIndex + HeadElement::SIZE, TailElements...>::isElementValid();
  }

public:
  inline constexpr typename HeadElement::DataType get() const {
    return HeadElement::deserialize(&_buffer[ByteIndex]);
  }
  inline static constexpr std::array<Byte, HeadElement::SIZE> serialize(const typename HeadElement::DataType value) {
    return HeadElement::serialize(value);
  }
};

template <std::size_t Index, std::size_t ByteIndex, class HeadElement, class... TailElements>
typename HeadElement::DataType get(const Message<Index, ByteIndex, HeadElement, TailElements...>& message) {
  return message.get();
}

} // end namespace impl

using StartByte = impl::ConstElement<Byte, UART::BUFFER::START_BYTE>;
using StopByte  = impl::ConstElement<Byte, UART::BUFFER::END_BYTE>;

template <Byte UartCommand>
struct Command : public impl::ConstElement<Byte, UartCommand> { };

struct Float  : public impl::Element<float> { };
struct Int16  : public impl::Element<std::int16_t> { };
struct Uint8  : public impl::Element<std::uint8_t> { };
struct Uint32 : public impl::Element<std::uint32_t> { };

} // end namespace element

template <class... Elements>
struct Message : public element::impl::Message<0u, 0u, Elements...>
{
protected:
  static constexpr std::size_t SIZE = (Elements::SIZE + ...);
  using element::impl::Message<0u, 0u, Elements...>::_buffer;

public:
  Message(const std::array<Byte, SIZE>& message_candidate)
    : element::impl::Message<0u, 0u, Elements...>()
  {
    _buffer = message_candidate;
  }

  template <typename... Arguments>
  Message(const Arguments... args) : element::impl::Message<0u, 0u, Elements...>(args...)
  { }

  inline static constexpr std::size_t size() { return SIZE; }
  inline constexpr const std::array<Byte, SIZE>& data() const { return _buffer; }
  inline constexpr bool isMessageCandidateValid() const {
    return element::impl::Message<0u, 0u, Elements...>::isElementValid();
  }
  template <std::size_t Index>
  inline constexpr auto getElementValue() const { return element::impl::get<Index>(*this); }
};

template <class CommandByte, class ...Elements>
struct MessageFrame : public Message<element::StartByte, CommandByte, Elements..., element::StopByte>
{
public:
  using Message<element::StartByte, CommandByte, Elements..., element::StopByte>::SIZE;

  MessageFrame(const typename Elements::DataType... args)
    : Message<element::StartByte, CommandByte, Elements..., element::StopByte>(0u, 0u, args..., 0u) // 0u are dummy for const values
  { }

  MessageFrame(const std::array<Byte, SIZE>& buffer)
    : Message<element::StartByte, CommandByte, Elements..., element::StopByte>(buffer)
  { }
};


} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
