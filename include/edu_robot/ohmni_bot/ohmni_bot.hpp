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
namespace ohmni_bot {

class OhmniBot : public robot::Robot
{
public:
  struct Parameter {
    struct {
      struct {
        float x = 0.25f;
        float y = 0.32f;
      } length;
      float wheel_diameter = 0.17f;
    } mecanum;
  };

  OhmniBot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface);
  ~OhmniBot() override;

  void initialize(eduart::robot::HardwareComponentFactory& factory) override;

protected:
  Eigen::MatrixXf getKinematicMatrix(const Mode mode) const override;
  // Needs to be called by derived classes.

  Parameter _parameter;
};

} // end namespace ohmni_bot
} // end namespace robot
} // end namespace eduart
