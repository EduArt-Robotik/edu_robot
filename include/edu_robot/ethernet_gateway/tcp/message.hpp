/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <edu_robot/rotation_per_minute.hpp>

#include <netinet/in.h>

#include <algorithm>
#include <array>
#include <tuple>
#include <type_traits>
#include <vector>

namespace eduart {
namespace robot {
namespace ethernet {
namespace tcp {
namespace message {

static constexpr std::size_t MAX_MESSAGE_SIZE = 200;
using TxMessageDataBuffer = std::vector<Byte>;
using RxMessageDataBuffer = std::vector<Byte>;

namespace element {

struct SequenceNumber;

namespace impl {

template <typename DataType>
struct DataField {
  inline static constexpr std::size_t size() { return sizeof(DataType); }
  using type = DataType;

  // It is a default serialization. On demand customization should take place in derived classes.
  inline static constexpr std::array<Byte, size()> serialize(const DataType value)
  {
    // Serialization of 1, 2 and 4 bytes sized data types are supported.
    if constexpr (size() == sizeof(Byte)) {
      return { value };
    }
    else if constexpr (size() == sizeof(std::uint16_t)) {
      const std::uint16_t host_order = *static_cast<const std::uint16_t*>(static_cast<const void*>(&value));
      const std::uint16_t network_order = ::htons(host_order);
      std::array<Byte, size()> serialized_bytes = { 0 };
    
      void* data_address = static_cast<void*>(serialized_bytes.data());
      *static_cast<std::uint16_t*>(data_address) = network_order;
      return serialized_bytes;
    }
    else if constexpr (size() == sizeof(std::uint32_t)) {
      const std::uint32_t host_order = *static_cast<const std::uint32_t*>(static_cast<const void*>(&value));
      const std::uint32_t network_order = ::htonl(host_order);
      std::array<Byte, size()> serialized_bytes = { 0 };
    
      void* data_address = static_cast<void*>(serialized_bytes.data());
      *static_cast<std::uint32_t*>(data_address) = network_order;
      return serialized_bytes;
    }
    // else
    static_assert((size() == 1 || size() % 2 == 0) && size() < 5, "Datatype is not supported.");
  }
  inline static constexpr DataType deserialize(const Byte data[size()])
  {
    if constexpr (size() == sizeof(Byte)) {
      return data[0];
    }
    else if constexpr (size() == sizeof(std::uint16_t)) {
      const std::uint16_t host_order = ::ntohs(*reinterpret_cast<const std::uint16_t*>(data));
      const void* data_address = static_cast<const void*>(&host_order);
      return *static_cast<const DataType*>(data_address);      
    }
    else if constexpr (size() == sizeof(std::uint32_t)) {
      const std::uint32_t host_order = ::ntohl(*reinterpret_cast<const std::uint32_t*>(data));
      const void* data_address = static_cast<const void*>(&host_order);
      return *static_cast<const DataType*>(data_address);       
    }
    // else
    static_assert((size() == 1 || size() % 2 == 0) && size() < 5, "Datatype is not supported.");
  }
  inline static constexpr bool isElementValid(const Byte[size()]) { return true; }
};

// (Partial-)Specialized Data Fields
template <typename DataType, DataType Value>
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
inline constexpr auto make_message_search_pattern(const Byte sequence_number, const std::tuple<Elements...>)
{
  constexpr std::size_t bytes = (std::tuple_element<Indices, std::tuple<Elements...>>::type::size() + ...);
  std::array<Byte, bytes> search_pattern = { 0 };
  auto it_search_pattern = search_pattern.begin();

  ([&]{
    if constexpr (std::is_same<typename std::tuple_element<Indices, std::tuple<Elements...>>::type, SequenceNumber>::value) {
      const auto element_pattern = std::tuple_element<Indices, std::tuple<Elements...>>::type::makeSearchPattern(sequence_number);
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

// Specializations of Message Elements
using StartByte = impl::ConstDataField<Byte, PROTOCOL::BUFFER::START_BYTE>;
using StopByte  = impl::ConstDataField<Byte, PROTOCOL::BUFFER::END_BYTE>;
struct SequenceNumber : public impl::DataField<Byte> {
  inline static constexpr std::array<Byte, SequenceNumber::size()> makeSearchPattern(const Byte sequence_number) {
    return impl::DataField<Byte>::serialize(sequence_number);
  } 
};

template <Byte TcpCommand>
struct Command : public impl::ConstDataField<Byte, TcpCommand> {
  inline static constexpr std::array<Byte, Command::size()> makeSearchPattern() {
    return impl::DataField<Byte>::serialize(TcpCommand | 0x80);
  }
};
template <Byte TcpCommand>
struct Response : public Command<TcpCommand | 0x80> { };

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
struct Uint16 : public impl::DataField<std::uint16_t> { };
struct Uint32 : public impl::DataField<std::uint32_t> { };

} // end namespace element

template <class... Elements>
struct Message : public std::tuple<Elements...>
{
  inline static constexpr std::size_t size() { return (Elements::size() + ...); }
  static TxMessageDataBuffer serialize(const typename Elements::type&... element_value) {
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
struct MessageFrame : public Message<element::StartByte, element::SequenceNumber, CommandByte, Elements..., element::StopByte>
{
private:
  using MessageType = Message<element::StartByte, element::SequenceNumber, CommandByte, Elements..., element::StopByte>;

public:
  using MessageType::size;

  inline static TxMessageDataBuffer serialize(
    const Byte sequence_number, const typename Elements::type&... element_value)
  {
    return MessageType::serialize(0, sequence_number, CommandByte::value(), element_value..., 0);
  }
  inline constexpr static auto makeSearchPattern(const Byte sequence_number) {
    return element::impl::make_message_search_pattern<0, 1, 2>(sequence_number, MessageType{});
  }
  template <std::size_t Index>
  inline constexpr static auto deserialize(const RxMessageDataBuffer& rx_buffer) {
    return MessageType::template deserialize<Index + 3>(rx_buffer); // \todo clarify if + 3 is a bug
  }
};

template <class MeasurementId, class ...Elements>
struct MeasurementFrame : public Message<element::StartByte, MeasurementId, Elements..., element::StopByte>
{
protected:
  using MessageType = Message<element::StartByte, MeasurementId, Elements..., element::StopByte>;

public:
  using MessageType::size;

  inline constexpr static auto makeSearchPattern() {
    return element::impl::make_message_search_pattern<0, 1>(MessageType{});
  }
  template <std::size_t Index>
  inline constexpr static auto deserialize(const RxMessageDataBuffer& rx_buffer) {
    return MessageType::template deserialize<Index + 2>(rx_buffer);
  }
};

} // end namespace message
} // end namespace tcp
} // end namespace ethernet
} // end namespace eduart
} // end namespace robot
