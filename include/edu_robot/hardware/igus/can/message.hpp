/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/igus/can/protocol.hpp"

#include <edu_robot/hardware/can/message.hpp>
#include <edu_robot/rpm.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace igus {
namespace can {
namespace message {

namespace element {

struct Velocity : public hardware::can::message::element::impl::DataField<std::uint8_t> {
  inline static constexpr std::array<hardware::can::message::Byte, size()> serialize(const Rpm value) {
    return hardware::can::message::element::impl::DataField<std::uint8_t>::serialize(
      static_cast<std::uint8_t>(value + 127.0)
    ); // 127 is zero
  }
  inline static constexpr Rpm deserialize(const hardware::can::message::Byte data[size()]) {
    return Rpm(hardware::can::message::element::impl::DataField<std::uint8_t>::deserialize(data) - 127); // 127 is zero
  }
};

} // end namespace element

} // end namespace message
} // end namespace can
} // end namespace igus
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
