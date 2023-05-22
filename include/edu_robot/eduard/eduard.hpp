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
  struct Parameter {
    struct {
      struct {
        float x = 0.25f;
        float y = 0.32f;
      } length;
      float wheel_diameter = 0.17f;
    } skid;
    struct {
      struct {
        float x = 0.25f;
        float y = 0.36f;
      } length;
      float wheel_diameter = 0.1f;
    } mecanum;    
  };

  Eduard(
    const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface, const std::string& ns = "");
  ~Eduard() override;

protected:
  Eigen::MatrixXf getKinematicMatrix(const Mode mode) const override;
  // Needs to be called by derived classes.
  void initialize(eduart::robot::HardwareComponentFactory& factory) override;

  Parameter _parameter;
};

} // end namespace eduard
} // end namespace robot
} // end namespace eduart
