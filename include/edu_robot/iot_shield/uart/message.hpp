/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/rotation_per_minute.hpp>

#include <algorithm>
#include <array>
#include <tuple>
#include <utility>
#include <cstdint>
#include <cstddef>

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

using TxMessageDataBuffer = std::array<std::uint8_t, UART::BUFFER::TX_SIZE>;
using RxMessageDataBuffer = std::array<std::uint8_t, uart::message::UART::BUFFER::RX_SIZE>;

namespace element {
namespace impl {

// inline constexpr bool is_big_endian() {
//   constexpr std::uint32_t integer = 0x01020304;
//   const void* address = static_cast<const void*>(&integer);
//   const std::uint8_t* bits = static_cast<const std::uint8_t*>(address);

//   return bits[0] == 1;
// }

template <typename DataType>
struct DataField {
  inline static constexpr std::size_t size() { return sizeof(DataType); }
  using type = DataType;

  // It is a default serialization. On demand customization should take place in derived classes.
  inline static constexpr std::array<Byte, size()> serialize(const DataType value)
  {
    std::array<Byte, size()> serialized_bytes = { 0 };
    void* data_address = static_cast<void*>(serialized_bytes.data());
    *static_cast<DataType*>(data_address) = value;

    return serialized_bytes;
  }
  inline static constexpr DataType deserialize(const Byte data[size()])
  {
    const void* data_address = static_cast<const void*>(data);
    return *static_cast<const DataType*>(data_address);
  }
  inline static constexpr bool isElementValid(const Byte[size()]) { return true; }
};

template <typename DataType, DataType Value>
struct ConstDataField : public DataField<DataType> {
  using DataField<DataType>::size;
  using type = typename DataField<DataType>::type;

  inline static constexpr DataType value() { return Value; }
  inline static constexpr std::array<Byte, size()> serialize(const DataType) {
    return DataField<DataType>::serialize(Value); }
  inline static constexpr bool isElementValid(const Byte data[size()]) {
    return Value == DataField<DataType>::deserialize(data);
  }
  inline static constexpr std::array<Byte, size()> makeSearchPattern() {
    return DataField<DataType>::serialize(Value); }
};

template <std::size_t Index, class Message>
struct element_byte_index;

template <std::size_t Index, class... Elements>
struct element_byte_index<Index, std::tuple<Elements...>> : element_byte_index<Index - 1, std::tuple<Elements...>> {
  constexpr static std::size_t value = element_byte_index<Index - 1, std::tuple<Elements...>>::value
                                     + element_byte_index<Index - 1, std::tuple<Elements...>>::size();
protected:
  inline constexpr static std::size_t size() { 
    return std::tuple_element<Index, std::tuple<Elements...>>::type::size(); }
};

template <class... Elements>
struct element_byte_index<0, std::tuple<Elements...>> {
  constexpr static std::size_t value = 0;
protected:
  inline constexpr static std::size_t size() { 
    return std::tuple_element<0, std::tuple<Elements...>>::type::size(); }
};
// /**
//  * \brief This class constructs a UART message based on the given message elements. Methods
//  *        to (de-)serialize and to check received message candidate are provided.
//  *
//  * \todo make a static version for received message checking --> no instantiation necessary.
//  */
// template <std::size_t Index, std::size_t ByteIndex, class... Elements>
// struct MessageElement;

// template <std::size_t Index, std::size_t ByteIndex>
// struct MessageElement<Index, ByteIndex>{ };

// template <std::size_t Index, std::size_t ByteIndex, class HeadElement, class... TailElements>
// struct MessageElement<Index, ByteIndex, HeadElement, TailElements...>
//   : public MessageElement<Index + 1, ByteIndex + HeadElement::size(), TailElements...>
// {
//   using ElementType = HeadElement;
// };

// template <std::size_t Index, class MessageType>
// struct message_element;

// template <std::size_t Index, std::size_t ByteIndex, class HeadElement, class... TailElements>
// struct message_element<Index, MessageElement<0, 0, HeadElement, TailElements...>> { using type = HeadElement; };

template <class... Elements>
constexpr TxMessageDataBuffer serialize(const typename Elements::type&... element_value, const std::tuple<Elements...>)
{
  TxMessageDataBuffer tx_buffer = { 0 };
  std::size_t byte_offset = 0;

  ([&]{
    const auto serialized_bytes = Elements::serialize(element_value);
    std::copy(serialized_bytes.begin(), serialized_bytes.end(), tx_buffer.begin() + byte_offset);    
    byte_offset += serialized_bytes.size();
  }(), ...);

  return tx_buffer;
}

template <std::size_t... Indices, class... Elements>
inline constexpr auto make_message_search_pattern(const std::tuple<Elements...>)
{
  constexpr std::size_t bytes = (std::tuple_element<Indices, std::tuple<Elements...>>::type::size() + ...);
  std::array<Byte, bytes> search_pattern = { 0 };
  auto it_search_pattern = search_pattern.begin();

  ([&]{
    const auto element_pattern = std::tuple_element<Indices, std::tuple<Elements...>>::type::makeSearchPattern();
    std::copy(element_pattern.begin(), element_pattern.end(), it_search_pattern);
    it_search_pattern += element_pattern.size();
  }(), ...);

  return search_pattern;
}

} // end namespace impl

using StartByte = impl::ConstDataField<Byte, UART::BUFFER::START_BYTE>;
using StopByte  = impl::ConstDataField<Byte, UART::BUFFER::END_BYTE>;

template <Byte UartCommand>
struct Command : public impl::ConstDataField<Byte, UartCommand> { };
struct Float  : public impl::DataField<float> { };
struct Int16  : public impl::DataField<std::int16_t> {
  inline static constexpr std::array<Byte, size()> serialize(const Rpm value) {
    return impl::DataField<std::int16_t>::serialize(static_cast<std::int16_t>(value * 100.0f + 0.5f));
  }
};
struct Uint8  : public impl::DataField<std::uint8_t> {
  inline static constexpr std::array<Byte, size()> serialize(const bool value) {
    return impl::DataField<std::uint8_t>::serialize(static_cast<std::uint8_t>(value));
  }
  inline static constexpr std::array<Byte, size()> serialize(const std::uint8_t value) {
    return impl::DataField<std::uint8_t>::serialize(value);
  }
};
struct Uint32 : public impl::DataField<std::uint32_t> { };

} // end namespace element

template <class... Elements>
struct Message : public std::tuple<Elements...>
{
  inline static constexpr std::size_t size() { return (Elements::size() + ...); }
  static constexpr TxMessageDataBuffer serialize(const typename Elements::type&... element_value) {
    return element::impl::serialize<Elements...>(element_value..., std::tuple<Elements...>{});
  }
  template <std::size_t Index>
  inline constexpr static typename std::tuple_element<Index, std::tuple<Elements...>>::type::type deserialize(
    const RxMessageDataBuffer& rx_buffer) {
      return std::tuple_element<Index, std::tuple<Elements...>>::type::deserialize(
        rx_buffer.data() + element::impl::element_byte_index<Index, std::tuple<Elements...>>::value
      );
  }
};

template <class CommandByte, class ...Elements>
struct MessageFrame : public Message<element::StartByte, CommandByte, Elements..., element::StopByte>
{
private:
  using MessageType = Message<element::StartByte, CommandByte, Elements..., element::StopByte>;

public:
  using MessageType::size;

  inline constexpr static TxMessageDataBuffer serialize(const typename Elements::type&... element_value) {
    return MessageType::serialize(0, CommandByte::value(), element_value..., 0);
  }
  inline constexpr static auto makeSearchPattern() {
    return element::impl::make_message_search_pattern<1>(MessageType{});
  }
};


} // end namespace message
} // end namespace uart
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot
