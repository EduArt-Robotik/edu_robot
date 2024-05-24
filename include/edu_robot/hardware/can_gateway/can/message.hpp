/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware/message_buffer.hpp>

#include <cstddef>
#include <cstdint>
#include <array>
#include <tuple>

#include <netinet/in.h>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {
namespace can {
namespace message {

static constexpr std::size_t MAX_MESSAGE_SIZE = 8;
using hardware::message::Byte;
using hardware::message::RxMessageDataBuffer;
using hardware::message::TxMessageDataBuffer;

namespace element {

struct CanAddress;

namespace impl {

template <typename DataType, bool BigEndian = true>
struct DataField {
  inline static constexpr std::size_t size() { return sizeof(DataType); }
  using type = DataType;

  // It is a default serialization. On demand customization should take place in derived classes.
  inline static constexpr std::array<Byte, size()> serialize(const DataType value)
  {
    std::array<Byte, size()> serialized_bytes = { 0 };

    // Serialization of 1, 2 and 4 bytes sized data types are supported.
    if constexpr (BigEndian) {
      const std::uint8_t* value_address = reinterpret_cast<const std::uint8_t*>(&value);

      for (std::size_t i = 0, j = size() - 1; i < size(); ++i, --j) {
        serialized_bytes[i] = value_address[j];
      }
    }
    else {
      // Do implementation for ARM only at the moment.
      void* data_address = static_cast<void*>(serialized_bytes.data());
      *static_cast<DataType*>(data_address) = value;
    }

    return serialized_bytes;
  }
  inline static constexpr DataType deserialize(const Byte data[size()])
  {
    if constexpr (BigEndian) {
      DataType value;
      std::uint8_t* value_address = reinterpret_cast<std::uint8_t*>(&value);

      for (std::size_t i = 0, j = size() - 1; i < size(); ++i, --j) {
        value_address[i] = data[j];
      }

      return value;
    }
    else {
      // Do implementation for ARM only at the moment.    
      const void* data_address = static_cast<const void*>(data);
      return *static_cast<const DataType*>(data_address);
    }
  }
  inline static constexpr bool isElementValid(const Byte[size()]) { return true; }
};

// (Partial-)Specialized Data Fields
template <typename DataType, DataType Value, bool BigEndian = true>
struct ConstDataField : public DataField<DataType> {
  using DataField<DataType>::size;
  using type = typename DataField<DataType>::type;

  inline static constexpr DataType value() { return Value; }
  inline static constexpr std::array<Byte, size()> serialize(const DataType) {
    return DataField<DataType>::serialize(Value);
  }
  inline static constexpr bool isElementValid(const Byte data[size()]) {
    return Value == DataField<DataType>::deserialize(data);
  }
  inline static constexpr std::array<Byte, size()> makeSearchPattern() {
    return DataField<DataType>::serialize(Value);
  }
};


// Helper for Message Element Handling
template <std::size_t Index, class Message>
struct element_byte_index;

template <std::size_t Index, class... Elements>
struct element_byte_index<Index, std::tuple<Elements...>> : element_byte_index<Index - 1, std::tuple<Elements...>> {
  constexpr static std::size_t value = element_byte_index<Index - 1, std::tuple<Elements...>>::value
                                     + element_byte_index<Index - 1, std::tuple<Elements...>>::size();
protected:
  inline constexpr static std::size_t size() { 
    return std::tuple_element<Index, std::tuple<Elements...>>::type::size();
  }
};

template <class... Elements>
struct element_byte_index<0, std::tuple<Elements...>> {
  constexpr static std::size_t value = 0;
protected:
  inline constexpr static std::size_t size() { 
    return std::tuple_element<0, std::tuple<Elements...>>::type::size();
  }
};

// Helper Functions for Message Handling
template <class... Elements>
TxMessageDataBuffer serialize(const typename Elements::type&... element_value, const std::tuple<Elements...>)
{
  constexpr std::size_t buffer_size = (Elements::size() + ...);
  TxMessageDataBuffer tx_buffer(buffer_size);
  std::size_t byte_offset = 0;

  ([&]{
    const auto serialized_bytes = Elements::serialize(element_value);
    std::copy(serialized_bytes.begin(), serialized_bytes.end(), tx_buffer.begin() + byte_offset);    
    byte_offset += serialized_bytes.size();
  }(), ...);

  return tx_buffer;
}

template <std::size_t... Indices, class... Elements>
inline constexpr auto make_message_search_pattern(const std::uint32_t can_address, const std::tuple<Elements...>)
{
  constexpr std::size_t bytes = (std::tuple_element<Indices, std::tuple<Elements...>>::type::size() + ...);
  std::array<Byte, bytes> search_pattern = { 0 };
  auto it_search_pattern = search_pattern.begin();

  ([&]{
    if constexpr (std::is_base_of<CanAddress, typename std::tuple_element<Indices, std::tuple<Elements...>>::type>::value) {
      const auto element_pattern = std::tuple_element<Indices, std::tuple<Elements...>>::type::makeSearchPattern(can_address);
      std::copy(element_pattern.begin(), element_pattern.end(), it_search_pattern);
      it_search_pattern += element_pattern.size();
    }
    else {
      const auto element_pattern = std::tuple_element<Indices, std::tuple<Elements...>>::type::makeSearchPattern();
      std::copy(element_pattern.begin(), element_pattern.end(), it_search_pattern);
      it_search_pattern += element_pattern.size();      
    }
  }(), ...);

  return search_pattern;
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

// Defining Basic Datatypes valid for all CAN Hardware
template <Byte CommandByte>
struct Command : public impl::ConstDataField<Byte, CommandByte> { };

struct CanAddress : public impl::DataField<std::uint32_t, false> {
  inline static constexpr std::array<Byte, CanAddress::size()> makeSearchPattern(const std::uint32_t can_address) {
    return impl::DataField<std::uint32_t, false>::serialize(can_address);
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

// Big Endian
using Int8 = impl::DataField<std::int8_t>;
using Int16 = impl::DataField<std::int16_t>;
using Int32 = impl::DataField<std::int32_t>;
using Uint16 = impl::DataField<std::uint16_t>;
using Uint24 = impl::DataField<std::array<unsigned char, 3>>;
using Uint32 = impl::DataField<std::uint32_t>;
using Float = impl::DataField<float>;

// Little Endian
using Int16LE = impl::DataField<std::int16_t, false>;
using Uint16LE = impl::DataField<std::uint16_t, false>;
using Uint24LE = impl::DataField<std::array<unsigned char, 3>, false>;
using FloatLE = impl::DataField<float, false>;

} // end namespace element

// Defining CAN Message and Message Frame
template <class... Elements>
struct Message : public std::tuple<Elements...>
{
  inline static constexpr std::size_t size() { return (Elements::size() + ...); }
  static TxMessageDataBuffer serialize(const typename Elements::type&... element_value) {
    return element::impl::serialize<Elements...>(element_value..., std::tuple<Elements...>{});
  }
  template <std::size_t Index, class DataBuffer>
  inline constexpr static typename std::tuple_element<Index, std::tuple<Elements...>>::type::type deserialize(
    const DataBuffer& rx_buffer)
  {
    return std::tuple_element<Index, std::tuple<Elements...>>::type::deserialize(
      rx_buffer.data() + element::impl::element_byte_index<Index, std::tuple<Elements...>>::value
    );
  }
  template <class DataBuffer>
  inline static constexpr bool hasCorrectLength(const DataBuffer& buffer) {
    return buffer.size() == size();
  } 
};

template <class... Elements>
struct MessageFrame : public Message<element::CanAddress, Elements...>
{
protected:
  using MessageType = Message<element::CanAddress, Elements...>;

public:
  using MessageType::size;

  inline static TxMessageDataBuffer serialize(
    const std::uint32_t can_address, const typename Elements::type&... element_value)
  {
    return MessageType::serialize(can_address, element_value...);
  }
  inline constexpr static auto makeSearchPattern(const std::uint32_t can_address) {
    return element::impl::make_message_search_pattern<0>(can_address, MessageType{});
  }
  template <std::size_t Index>
  inline constexpr static auto deserialize(const RxMessageDataBuffer& rx_buffer) {
    return MessageType::template deserialize<Index>(rx_buffer);
  }
  inline static constexpr std::uint32_t canId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
};

template <class... Elements>
struct NoResponseMessageFrame : public MessageFrame<Elements...>
{
  // make a empty search pattern to indicate there is no response
  inline constexpr static auto makeSearchPattern(const std::uint32_t can_address) {
    (void)can_address;
    return std::array<Byte, 0>();
  }
};

} // end namespace message
} // end namespace can
} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
