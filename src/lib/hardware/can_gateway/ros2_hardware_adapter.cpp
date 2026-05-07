#include "edu_robot/hardware/can_gateway/ros2_hardware_adapter.hpp"

#include <lgpio.h>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

static int resolve_resistor_mode(const std::string& mode) {
  static std::unordered_map<std::string, int> mapping_resistor_mode = {
    {"pull_up"  , LG_SET_PULL_UP},
    {"pull_down", LG_SET_PULL_DOWN},
    {"none"     , 0}
  };
  
  return mapping_resistor_mode.at(mode);
}

Ros2HardwareAdapter::~Ros2HardwareAdapter()
{

}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::SystemInterface::on_init(info);
  _gpio_handle = lgGpiochipOpen(0);

  return  _gpio_handle < 0 ? hardware_interface::CallbackReturn::ERROR : hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;

  for (const auto& gpio : info_.gpios) {
    // outputs
    for (const auto& command : gpio.command_interfaces) {
      // get pin number from parameter
      try {
        if (command.name.find("pwm") != std::string::npos) {
          // pwm pin
          const int pin = std::stoi(command.parameters.at("pin"));
          const int frequency = std::stoi(command.parameters.at("frequency"));
          const float duty_cycle = std::stof(command.parameters.at("initial_value"));

          if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
            throw std::invalid_argument("initial_value for pwm must be between 0.0 and 1.0");
          }
          if (frequency <= 0) {
            throw std::invalid_argument("frequency for pwm must be greater than 0");
          }

          // configure, pwm output, with given frequency and initial duty cycle, no offset and continuous mode
          if (lgTxPwm(_gpio_handle, pin, frequency, duty_cycle * 100.0f, 0, 0) < 0) {
            RCLCPP_ERROR(get_logger(), "could not claim GPIO pin %d for PWM output", pin);
          }
          else {
            RCLCPP_INFO(
              get_logger(), "claimed GPIO pin %d for PWM output with frequency %d and initial duty cycle %f",
              pin, frequency, duty_cycle
            );
            _gpio_pin[command.name] = pin;
            _gpio_initial_value[command.name] = duty_cycle;
            _pwm_frequency[command.name] = frequency;
          }
        }
        else if (command.name.find("dout") != std::string::npos) {
          // digital output pin
          const int pin = std::stoi(command.parameters.at("pin"));
          const bool initial_value = static_cast<bool>(std::stoi(command.parameters.at("initial_value")));

          // configure, output, no resistor, initial value
          if (lgGpioClaimOutput(_gpio_handle, 0, pin, initial_value) < 0) {
            RCLCPP_ERROR(get_logger(), "could not claim GPIO pin %d for output", pin);
          }
          else {
            RCLCPP_INFO(get_logger(), "claimed GPIO pin %d for output with initial value %d", pin, initial_value);
            _gpio_pin[command.name] = pin;
            _gpio_initial_value[command.name] = initial_value;
          }
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
          const auto resistor = resolve_resistor_mode(state.parameters.at("resistor"));
          
          // configure
          if (lgGpioClaimInput(_gpio_handle, resistor, pin) < 0) {
            RCLCPP_ERROR(get_logger(), "could not claim GPIO pin %d for input", pin);
          }
          else {
            RCLCPP_INFO(
              get_logger(), "claimed GPIO pin %d for input with resistor mode %s",
              pin, state.parameters.at("resistor").c_str()
            );
            _gpio_pin[state.name] = pin;
          }
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "activating hardware and set it to default value");

  // Setting Output Default Values, also Reflecting back State
  for (const auto& [name, pin] : _gpio_pin) {
    if (name.find("pwm") != std::string::npos) { 
      const float initial_value = _gpio_initial_value.at(name);
      const float frequency = _pwm_frequency.at(name);
      lgTxPwm(_gpio_handle, pin, frequency, initial_value * 100.0f, 0, 0);
      set_command(name, initial_value);
      set_state(name, initial_value);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "deactivating hardware by freeing all pins.");

  for (const auto& [name, pin] : _gpio_pin) {
    lgGpioFree(_gpio_handle, pin);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ros2HardwareAdapter::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  for (const auto& [name, description] : gpio_state_interfaces_) {
    try {
      if (name.find("din") != std::string::npos || name.find("dout") != std::string::npos) {
        // digital io
        const int pin = _gpio_pin.at(name);
        const int value = lgGpioRead(_gpio_handle, pin);
        set_state(name, static_cast<double>(value));
      }
      else if (name.find("pwm") != std::string::npos) {
        // reflect pwm command as state
        set_state(name, get_command(name));
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
  (void)time;
  (void)period;

  for (const auto& [name, description] : gpio_command_interfaces_) {
    try {
      if (name.find("dout") != std::string::npos) {
        // digital output
        if (lgGpioWrite(_gpio_handle, _gpio_pin.at(name), get_command(name)) < 0) {
          throw std::runtime_error("failed to write value to gpio pin");
        }
      }
      else if (name.find("pwm") != std::string::npos) {
        // pwm output
        const float duty_cycle = get_command(name);
        const int frequency = _pwm_frequency.at(name);
        const int pin = _gpio_pin.at(name);

        if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
          throw std::invalid_argument("duty_cycle for pwm must be between 0.0 and 1.0");
        }
        if (lgTxPwm(_gpio_handle, pin, frequency, duty_cycle * 100.0f, 0, 0) < 0) {
          throw std::runtime_error("failed to write value to pwm gpio pin");
        }
      
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
