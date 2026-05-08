/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
 #pragma once

#include <hardware_interface/system_interface.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

class Ros2HardwareAdapter : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ros2HardwareAdapter)

  ~Ros2HardwareAdapter() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int _gpio_handle = -1;
  std::unordered_map<std::string, int> _gpio_pin;
  std::unordered_map<std::string, float> _gpio_initial_value;
  std::unordered_map<std::string, float> _pwm_frequency;
};

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
