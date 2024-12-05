/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware/ethernet_gateway/udp/protocol.hpp"
#include "edu_robot/hardware/ethernet_gateway/udp/message.hpp"

#include <Eigen/Dense>
#include <Eigen/src/Geometry/Quaternion.h>

#include <edu_robot/rpm.hpp>
#include <edu_robot/color.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace ethernet {
namespace udp {
namespace message {

using hardware::ethernet::udp::message::MessageFrame;

// Request Firmware Version
using GetFirmwareVersion = MessageFrame<element::Command<PROTOCOL::COMMAND::GET::FIRMWARE_VERSION>>;
using GetImuMeasurement = MessageFrame<element::Command<PROTOCOL::COMMAND::GET::IMU_MEASUREMENT>>;
using GetStatus = MessageFrame<element::Command<PROTOCOL::COMMAND::GET::STATUS>>;
using GetDistanceMeasurement = MessageFrame<element::Command<PROTOCOL::COMMAND::GET::DISTANCE_MEASUREMENT>,
                                            element::Uint8>; // Distance Sensor ID

// Set Motor Controller Parameters
using SetMotorControllerParameter = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_CONTROLLER_PARAMETER>,
                                                 element::Uint8,   // Motor ID
                                                 element::Uint8,   // CAN ID
                                                 element::Float,   // Gear Ratio
                                                 element::Float,   // Max RPM
                                                 element::Float,   // Threshold Time Stalled Detection
                                                 element::Float,   // Input Filter Weight
                                                 element::Uint32,  // Frequency
                                                 element::Uint16>; // Timeout (ms)

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
using SetMotorMeasurement = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::MOTOR_MEASUREMENT>,
                                         element::Uint8>; // Enable Flag
using SetImuMeasurement = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::IMU_MEASUREMENT>,
                                       element::Uint8>; // Enable Flag
using SetImuParameter = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::IMU_PARAMETER>,
                                     element::Uint8,  // Enable Submitting Raw Data
                                     element::Float,  // Fusion Weight
                                     element::Float,  // Mounting Orientation Roll Angle
                                     element::Float,  // Mounting Orientation Pitch Angle
                                     element::Float>; // Mounting Orientation Yaw Angle
using SetDistanceSensorMeasurement = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::DISTANCE_SENSOR_MEASUREMENT>,
                                                  element::Uint8>; // Enable Flag
using SetDisableAllMeasurements = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::DISABLE_ALL_MEASUREMENTS>>;
using SetLighting = MessageFrame<element::Command<PROTOCOL::COMMAND::SET::LIGHTING_COLOR_AND_MODE>,
                                 element::Uint8,  // lighting mode
                                 element::Uint8,  // r value
                                 element::Uint8,  // g value
                                 element::Uint8>; // b value

// Responses
template <Byte TcpCommand>
struct Acknowledgement : public MessageFrame<element::Response<TcpCommand>, element::Uint8> {
  inline static constexpr bool wasAcknowledged(const RxMessageDataBuffer& rx_buffer) {
    return MessageFrame<element::Response<TcpCommand>, element::Uint8>::template deserialize<0>(rx_buffer);
  }
};

struct AcknowledgedMotorRpm : public MessageFrame<element::Response<PROTOCOL::COMMAND::SET::MOTOR_RPM>,
                                                  element::Float, // Motor 0 Current RPM
                                                  element::Float, // Motor 1 Current RPM
                                                  element::Uint8> // Motor Enabled Flag
{
  inline static constexpr Rpm rpm0(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
  inline static constexpr Rpm rpm1(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr bool enabled(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
};

struct AcknowledgedImuMeasurement : MessageFrame<element::Response<PROTOCOL::COMMAND::GET::IMU_MEASUREMENT>,
                                                 element::Float,  // linear velocity x
                                                 element::Float,  // linear velocity y
                                                 element::Float,  // linear velocity z
                                                 element::Float,  // angular velocity x
                                                 element::Float,  // angular velocity y
                                                 element::Float,  // angular velocity z
                                                 element::Float,  // orientation x
                                                 element::Float,  // orientation y
                                                 element::Float,  // orientation z
                                                 element::Float>{ // orientation w
  inline static Eigen::Vector3d angularVelocity(const RxMessageDataBuffer& rx_buffer) {
    return { deserialize<3>(rx_buffer), deserialize<4>(rx_buffer), deserialize<5>(rx_buffer) };
  }
  inline static Eigen::Vector3d linearAcceleration(const RxMessageDataBuffer& rx_buffer) {
    return { deserialize<0>(rx_buffer), deserialize<1>(rx_buffer), deserialize<2>(rx_buffer) };
  }
  inline static Eigen::Quaterniond orientation(const RxMessageDataBuffer& rx_buffer) {
    return { deserialize<9>(rx_buffer), deserialize<6>(rx_buffer), deserialize<7>(rx_buffer), deserialize<8>(rx_buffer) };
  }    
};                                                 

struct AcknowledgedDistanceMeasurement : MessageFrame<element::Response<PROTOCOL::COMMAND::GET::DISTANCE_MEASUREMENT>,
                                                      element::Float> { // distance measurement in meter
  inline static constexpr float distance(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
};

struct AcknowledgedStatus : MessageFrame<element::Response<PROTOCOL::COMMAND::GET::STATUS>,
                                         element::Float, // temperature
                                         element::Float, // voltage
                                         element::Float, // current
                                         element::Uint8> // status emergency button
{
  inline constexpr static float temperature(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }
  inline constexpr static float voltage(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline constexpr static float current(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }
  inline constexpr static bool statusEmergencyButton(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<3>(rx_buffer);
  }
};     

// Measurements
struct RpmMeasurement : public MeasurementFrame<element::Command<PROTOCOL::MEASUREMENT::MOTOR_CONTROLLER_RPM>,
                                                element::Uint8,
                                                element::Float,
                                                element::Float> {
  inline constexpr static std::uint8_t canId(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<0>(rx_buffer);
  }                                                  
  inline static constexpr Rpm rpm0(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<1>(rx_buffer);
  }
  inline static constexpr Rpm rpm1(const RxMessageDataBuffer& rx_buffer) {
    return deserialize<2>(rx_buffer);
  }  
};

} // end namespace message
} // end namespace udp
} // end namespace ethernet
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
