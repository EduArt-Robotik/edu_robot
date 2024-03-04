/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_robot/hardware_component_interfaces.hpp>
#include <edu_robot/motor.hpp>
#include <edu_robot/hardware_interface.hpp>

#include <edu_robot/diagnostic/diagnostic_component.hpp>
#include <edu_robot/diagnostic/standard_deviation.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/clock.hpp>

#include <memory>
#include <mutex>

namespace eduart {
namespace robot {

/**
 * \brief Represents a hardware motor controller with n motors. This class expects finished initialized motors at construction.
 *        This class needs to be realized by a specific hardware layer.
 */
class MotorController : public diagnostic::DiagnosticComponent
{
public:
  friend class action::CheckIfMotorIsEnabled;

  /**
   * \brief Hardware interface used for communication with actual hardware.
   */
  class HardwareInterface : public eduart::robot::HardwareInterface
                          , public HardwareComponent<Motor::Parameter>
                          , public HardwareActuator<std::vector<Rpm>>
                          , public HardwareSensor<const std::vector<Rpm>, const bool>
  {
  protected:
    HardwareInterface(const std::string& name, const std::size_t num_motors)
      : eduart::robot::HardwareInterface(HardwareInterface::Type::MOTOR_CONTROLLER)
      , _name(name), _num_motors(num_motors) { }

  public:
    inline std::size_t motors() const { return _num_motors; }
    inline const std::string& name() const { return _name; }

  private:
    std::string _name;
    std::size_t _num_motors;
  };                       

  /**
   * \brief Constructs this motor controller class using finished initialized motors and hardware interface.
   * \param name Name of the controller.
   * \param id Motor ID.
   * \param motors Finished initialized motors that are controlled by this controller class.
   * \param ros_node ROS node instance.
   * \param hardware_interface Interface to the hardware instance used to communication.
   */
  MotorController(const std::string& name, const std::size_t id, std::vector<Motor>&& motors,
    rclcpp::Node& ros_node, std::shared_ptr<HardwareInterface> hardware_interface);
  virtual ~MotorController();

  inline const std::string& name() const { return _name; }
  inline std::size_t id() const { return _id; }
  void setRpm(const std::vector<Rpm>& rpm);
  inline const std::vector<Rpm>& getMeasuredRpm() const {
    std::lock_guard guard(_mutex_access_data);
    return _measured_rpm;
  }
  inline bool isEnabled() const {
    std::lock_guard guard(_mutex_access_data);
    bool enabled = true;

    for (const auto& motor : _motor) {
      enabled &= motor.isEnabled();
    }

    return enabled;
  }
  inline std::size_t motors() const { return _motor.size(); }
  inline const Motor& motor(const std::size_t index) const { return _motor[index]; }
  void initialize() {
    if (_motor.empty()) {
      throw std::runtime_error("No motor is present --> can't initialize motors.");
    }

    _hardware_interface->initialize(_motor[0].parameter());
  }

private:
  void processMeasurementData(const std::vector<Rpm>& rpm, const bool enabled_flag);
  diagnostic::Diagnostic processDiagnosticsImpl() override;

  mutable std::mutex _mutex_access_data;

  std::string _name;
  std::size_t _id;
  std::vector<Motor> _motor;
  std::vector<Rpm> _measured_rpm;
  std::shared_ptr<HardwareInterface> _hardware_interface;

  // diagnostic
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _stamp_last_measurement;  
  rclcpp::Time _last_processing;
  std::shared_ptr<diagnostic::StandardDeviationDiagnostic<std::int64_t, std::greater<std::int64_t>>> _processing_dt_statistic;
  std::atomic_bool _lost_enable = false;
};

class HardwareComponentFactory;

std::vector<std::shared_ptr<MotorController>> helper_create_motor_controller(
  const HardwareComponentFactory& factory, const std::vector<std::string>& motor_name,
  const std::vector<std::string>& motor_joint_name, const std::string& robot_name, rclcpp::Node& ros_node);

} // end namespace robot
} // end namespace eduart
