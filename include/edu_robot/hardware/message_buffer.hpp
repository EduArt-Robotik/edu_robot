/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <vector>
#include <cstdint>

namespace eduart {
namespace robot {
namespace hardware {
namespace message {

using Byte = std::uint8_t;
using TxMessageDataBuffer = std::vector<Byte>;
using RxMessageDataBuffer = std::vector<Byte>;

} // end namespace message
} // end namespace hardware
} // end namespace robot
} // end namespace eduart