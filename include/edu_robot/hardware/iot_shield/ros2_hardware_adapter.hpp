/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
 #pragma once

#include <hardware_interface/system_interface.hpp>

#include <mraa/gpio.hpp>
#include <mraa/spi.hpp>
#include <mraa/i2c.hpp>
#include <mraa/pwm.hpp>
#include <mraa/aio.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

class Ros2HardwareAdapter : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ros2HardwareAdapter);

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
  std::unordered_map<std::string, std::shared_ptr<mraa::Gpio>> _gpio;
  std::unordered_map<std::string, std::shared_ptr<mraa::Pwm>> _pwm;
  std::unordered_map<std::string, std::shared_ptr<mraa::Aio>> _aio;
  std::shared_ptr<mraa::Spi> _spi;
  std::shared_ptr<mraa::I2c> _i2c;
};

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot
