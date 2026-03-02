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

struct COLOR {
  struct DEFAULT {
    static constexpr Color HEAD = { 0xff, 0xff, 0xff };
    static constexpr Color BACK = { 0xff, 0x00, 0x00 };
  };
};

struct WHEEL {
  enum TYPE {
    offroad,  // this is equivalent to skid
    mecanum
  };

  static std::string to_string(TYPE t) {
    switch(t) {
      case offroad: return "offroad";
      default:      return "mecanum";
    }
  }

  static TYPE from_string(const std::string &s) {
    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return std::tolower(c);
    });

    if (lower == "offroad") return offroad;
    else return mecanum;
  }
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
    WHEEL::TYPE wheel_type = WHEEL::TYPE::mecanum;    
  };

  Eduard(
    const std::string& robot_name, std::unique_ptr<HardwareRobotInterface> hardware_interface, const std::string& ns = "");
  ~Eduard() override;

protected:
  Eigen::MatrixXf getKinematicMatrix(const DriveKinematic kinematic) const override;
  // Needs to be called by derived classes.
  void initialize(eduart::robot::HardwareComponentFactory& factory) override;

  Parameter _parameter;
};

} // end namespace bot
} // end namespace robot
} // end namespace eduart
