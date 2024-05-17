/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/sensor_imu.hpp>

#include "edu_robot/hardware/communicator_device_interfaces.hpp"

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class ImuSensorHardware : public SensorImu::SensorInterface
                        , public CommunicatorTxRxDevice
{
public:
  ImuSensorHardware(const std::uint32_t can_id, std::shared_ptr<Communicator> communicator);
  ~ImuSensorHardware() override = default;

  void initialize(const SensorImu::Parameter& parameter) override;

private:
  void processRxData(const message::RxMessageDataBuffer& data);

  std::uint32_t _can_id;
  
  struct {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    std::array<bool, 3> valid_element;

    inline void gotOrientation() { valid_element[0] = true; }
    inline void gotLinearAcceleration() { valid_element[1] = true; }
    inline void gotAngularVelocity() { valid_element[2] = true; }
    inline void clear() {
      valid_element = { false, false, false };
    }
    inline bool complete() {
      return valid_element[0] && valid_element[1] && valid_element[2];
    }
  } _processing_data;  
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
