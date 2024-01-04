/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/robot.hpp>

#include <edu_robot/hardware_component_factory.hpp>

#include <memory>
#include <string>

namespace eduart {
namespace robot {
namespace universal_bot {

struct COLOR {
  struct DEFAULT {
    static constexpr Color HEAD = { 0xff, 0xff, 0xff };
    static constexpr Color BACK = { 0xff, 0x00, 0x00 };
  };
};

class UniversalBot : public robot::Robot
{
public:
  struct Parameter {
    struct Axis {
      struct {
        float x = 0.0f;
        float y = 0.32f;
      } length;
      float wheel_diameter = 0.17f;
    };

    std::vector<Axis> axis;
  };

  UniversalBot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface);
  ~UniversalBot() override;

protected:
  Eigen::MatrixXf getKinematicMatrix(const DriveKinematic kinematic) const override;
  // Needs to be called by derived classes.
  void initialize(eduart::robot::HardwareComponentFactory& factory) override;

  Parameter _parameter;
};

} // end namespace universal_bot
} // end namespace robot
} // end namespace eduart
