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
namespace bot {

class RebelMove : public robot::Robot
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

  RebelMove(
    const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns = "");
  ~RebelMove() override;

protected:
  Eigen::MatrixXf getKinematicMatrix(const DriveKinematic kinematic) const override;
  // Needs to be called by derived classes.
  void initialize(eduart::robot::HardwareComponentFactory& factory) override;

  Parameter _parameter;
};

} // end namespace bot
} // end namespace robot
} // end namespace eduart
