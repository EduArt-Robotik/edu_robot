/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/hardware_component_interface.hpp"
#include <edu_robot/robot.hpp>

#include <Eigen/Dense>

#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace eduard {

struct COLOR {
  struct DEFAULT {
    static constexpr Color HEAD = { 0xff, 0xff, 0xff };
    static constexpr Color BACK = { 0xff, 0x00, 0x00 };
  };
};

class Eduard : public robot::Robot
{
public:
  Eduard(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface);
  ~Eduard() override;

protected:
  void initialize(std::map<std::string, std::shared_ptr<HardwareComponentInterface<Color, Lighting::Mode>>> lightings_hardware,
                  std::map<std::string, std::shared_ptr<HardwareComponentInterface<Rpm>>> motor_controller_hardware,
                  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Rpm>>> motor_sensor_hardware,
                  std::map<std::string, std::shared_ptr<HardwareSensorInterface<float>>> range_sensor_hardware,
                  std::map<std::string, std::shared_ptr<HardwareSensorInterface<Eigen::Quaterniond>>> imu_sensor_hardware);
};

} // end namespace eduard
} // end namespace robot
} // end namespace eduart
