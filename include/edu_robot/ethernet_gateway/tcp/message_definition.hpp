/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/ethernet_gateway/tcp/protocol.hpp"
#include "edu_robot/ethernet_gateway/tcp/message.hpp"

#include <Eigen/Dense>
#include <array>
#include <edu_robot/rotation_per_minute.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace ethernet {
namespace tcp {
namespace message {

// Request Firmware Version
using GetFirmwareVersion = MessageFrame<element::Command<PROTOCOL::COMMAND::GET::FIRMWARE_VERSION>>;

} // end namespace message
} // end namespace ethernet
} // end namespace iotbot
} // end namespace eduart
} // end namespace robot