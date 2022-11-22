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

// Set Motor Controller Parameters
using SetMotorControllerParameter = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>,
                                                 element::Uint8,   // Motor ID
                                                 element::Uint8,   // CAN ID
                                                 element::Float,   // Gear Ratio
                                                 element::Float,   // Max RPM
                                                 element::Float,   // Threshold Time Stalled Detection
                                                 element::Float,   // Input Filter Weight
                                                 element::Uint32>; // Frequency Scale

using SetEncoderParameter = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::ENCODER_PARAMETER>,
                                                          element::Uint8,  // Motor ID
                                                          element::Uint8,  // CAN ID
                                                          element::Uint32, // Ticks per Rotation
                                                          element::Float,  // Input Filter Weight
                                                          element::Uint8>; // Inverted Flag

using SetPidControllerParameter = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::PID_CONTROLLER_PARAMETER>,
                                               element::Uint8,  // Motor ID
                                               element::Uint8,  // CAN ID
                                               element::Float,  // Kp
                                               element::Float,  // Ki
                                               element::Float,  // Kd
                                               element::Float,  // Lower Limit
                                               element::Float,  // Upper Limit
                                               element::Float,  // Input Filter Weight
                                               element::Uint8>; // Use Anti Windup Flag

using SetMotorRpm = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_RPM>,
                                 element::Uint8,  // CAN ID
                                 element::Float,  // Set Point RPM Motor 0
                                 element::Float>; // Set Point RPM Motor 1

using SetMotorEnabled = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_ENABLE>>;
using SetMotorDisabled = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_DISABLE>>;                                 

// Responses
template <Byte TcpCommand>
struct Acknowledgement : public MessageFrame<element::Response<TcpCommand>, element::Uint8> {
  inline static constexpr bool wasAcknowledged(const RxMessageDataBuffer& rx_buffer) {
    return MessageFrame<element::Response<TcpCommand>, element::Uint8>::template deserialize<2>(rx_buffer);
  }
};

} // end namespace message
} // end namespace tcp
} // end namespace ethernet
} // end namespace eduart
} // end namespace robot