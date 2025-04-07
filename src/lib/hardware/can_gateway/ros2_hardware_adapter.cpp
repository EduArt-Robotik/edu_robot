#include "edu_robot/hardware/can_gateway/ros2_hardware_adapter.hpp"

#include <iostream>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

static unsigned int resolve_resistor_mode(const std::string& mode) {
  static std::unordered_map<std::string, unsigned int> mapping_resistor_mode = {
    {"pull_up"  , PI_PUD_UP},
    {"pull_down", PI_PUD_DOWN},
    {"none"     , PI_PUD_OFF}
  };
  
  return mapping_resistor_mode.at(mode);
}

Ros2HardwareAdapter::~Ros2HardwareAdapter()
{
  gpioTerminate();
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::SystemInterface::on_init(info);
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);

  return gpioInitialise() < 0 ? hardware_interface::CallbackReturn::ERROR : hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);

  for (const auto& gpio : info_.gpios) {
    // outputs
    for (const auto& command : gpio.command_interfaces) {
      // get pin number from parameter
      try {
        if (command.name.find("pwm") != std::string::npos) {
          // pwm pin
          const int pin = std::stoi(command.parameters.at("pin"));
          const unsigned int period_us = std::stoul(command.parameters.at("period_us"));
          const unsigned int frequency = static_cast<unsigned int>(1.0 / (period_us * 1e-6));

          if (gpioSetPWMfrequency(pin, frequency) == PI_BAD_USER_GPIO) {
            throw std::runtime_error("Can't setup PWM channel.");
          }
          if (gpioPWM(pin, 0) != 0) {
            throw std::runtime_error("Can't write value on PWM pin.");
          }

          _pwm[gpio.name + '/' + command.name] = pin;
        }
        else if (command.name.find("dout") != std::string::npos) {
          // digital output pin
          const int pin = std::stoi(command.parameters.at("pin"));

          if (gpioSetMode(pin, PI_OUTPUT) != 0) {
            throw std::runtime_error("Can't set gpio to output.");
          }

          _gpio[gpio.name + '/' + command.name] = pin;
        }
      }
      catch (std::out_of_range& ex) {
        RCLCPP_ERROR(
          get_logger(), "urdf gpio description \"%s/%s\" is missing a parameter",
          gpio.name.c_str(), command.name.c_str()
        );
      }
      catch (std::exception& ex) {
        RCLCPP_ERROR(
          get_logger(), "error during initializing of port \"%s/%s\". what = %s.",
          gpio.name.c_str(), command.name.c_str(), ex.what()
        );
      }
    }

    // inputs
    for (const auto& state : gpio.state_interfaces) {
      try {
        if (state.name.find("din") != std::string::npos) {
          // digital input
          const int pin = std::stoi(state.parameters.at("pin"));
          const auto resistor_mode = state.parameters.at("resistor");

          if (gpioSetMode(pin, PI_INPUT) != 0) {
            throw std::runtime_error("Can't set gpio to input");
          }
          if (gpioSetPullUpDown(pin, resolve_resistor_mode(resistor_mode)) != 0) {
            throw std::runtime_error("Can't set resistor mode");
          }

          _gpio[gpio.name + '/' + state.name] = pin;
        }
      }
      catch (std::out_of_range& ex) {
        RCLCPP_ERROR(
          get_logger(), "urdf gpio description \"%s/%s\" is missing a parameter",
          gpio.name.c_str(), state.name.c_str()
        );
      }
      catch (std::exception& ex) {
        RCLCPP_ERROR(
          get_logger(), "error during initializing of port \"%s/%s\". what = %s.",
          gpio.name.c_str(), state.name.c_str(), ex.what()
        );
      }       
    }
  }

  std::cout << "pwm:\n";
  for (const auto& [name, egal] : _pwm) {
    std::cout << name << std::endl;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "activating hardware and set it to default value");

  // Setting Output Default Values, also Reflecting back State
  for (const auto& [name, pin] : _gpio) {
    if (gpioGetMode(pin) == PI_OUTPUT) { 
      gpioWrite(pin, 0);
      set_command(name, 0);
      set_state(name, 0);
    }
  }
  for (const auto& [name, pin] : _pwm) {
    gpioPWM(pin, 0);
    set_command(name, 0.0);
    set_state(name, 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "deactivating hardware.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ros2HardwareAdapter::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  for (const auto& [name, description] : gpio_state_interfaces_) {
    try {
      if (name.find("din") != std::string::npos) {
        // digital input
        set_state(name, gpioRead(_gpio.at(name)));
      }
      else if (name.find("pwm") != std::string::npos) {
        set_state(name, gpioGetPWMdutycycle(_pwm.at(name)));
      }
      else if (name.find("dout") != std::string::npos) {
        set_state(name, gpioRead(_gpio.at(name)));
      }
    }
    catch (std::out_of_range& ex) {
      RCLCPP_ERROR(get_logger(), "could not get state for \"%s\". No hardware driver present.", name.c_str());
    }
    catch (std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "error occurred during port operation on \"%s\". what = %s.", name.c_str(), ex.what());
    }    
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros2HardwareAdapter::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  (void)time;
  (void)period;

  for (const auto& [name, description] : gpio_command_interfaces_) {
    try {
      if (name.find("dout") != std::string::npos) {
        // digital output
        gpioWrite(_gpio.at(name), get_command(name));
      }
      else if (name.find("pwm") != std::string::npos) {
        // pwm output
        gpioPWM(_pwm.at(name), get_command(name));
      }
    }
    catch (std::out_of_range& ex) {
      RCLCPP_ERROR(get_logger(), "could not set state for \"%s\". No hardware driver present.", name.c_str());
    }
    catch (std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "error occurred during port operation on \"%s\". what = %s.", name.c_str(), ex.what());
    }
  }

  return hardware_interface::return_type::OK;
}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace eduart
} // end namespace robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(eduart::robot::hardware::can_gateway::Ros2HardwareAdapter, hardware_interface::SystemInterface)