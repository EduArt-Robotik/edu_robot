#include "edu_robot/hardware/can_gateway/ros2_hardware_adapter.hpp"

#include <iostream>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

static std::bitset<32> resolve_resistor_mode(const std::string& mode) {
  static std::unordered_map<std::string, std::bitset<32>> mapping_resistor_mode = {
    {"pull_up"  , gpiod::line_request::FLAG_BIAS_PULL_UP},
    {"pull_down", gpiod::line_request::FLAG_BIAS_PULL_DOWN},
    {"none"     , gpiod::line_request::FLAG_BIAS_DISABLE}
  };
  
  return mapping_resistor_mode.at(mode);
}

Ros2HardwareAdapter::~Ros2HardwareAdapter()
{

}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::SystemInterface::on_init(info);
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);

  _chip = std::make_shared<gpiod::chip>("gpiochip0", gpiod::chip::OPEN_BY_NAME);

  return  _chip == nullptr ? hardware_interface::CallbackReturn::ERROR : hardware_interface::CallbackReturn::SUCCESS;
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
          // const int pin = std::stoi(command.parameters.at("pin"));
          // const unsigned int period_us = std::stoul(command.parameters.at("period_us"));
          // const unsigned int frequency = static_cast<unsigned int>(1.0 / (period_us * 1e-6));

          RCLCPP_ERROR(get_logger(), "pwm is not supported at the moment");
        }
        else if (command.name.find("dout") != std::string::npos) {
          // digital output pin
          const int pin = std::stoi(command.parameters.at("pin"));
          const std::string pin_name = "GPIO" + std::to_string(pin);
          auto line = _chip->find_line(pin_name);

          // configure
          line.set_direction_output(0);
          line.set_flags(0);

          _gpio[gpio.name + '/' + command.name] = line;
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
          const std::string pin_name = "GPIO" + std::to_string(pin);
          auto line = _chip->find_line(pin_name);
          
          // configure
          line.set_direction_input();
          line.set_flags(resistor);

          _gpio[gpio.name + '/' + state.name] = line;
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
  for (const auto& [name, line] : _gpio) {
    if (line.direction() == gpiod::line::DIRECTION_OUTPUT) { 
      line.set_value(0);
      set_command(name, 0);
      set_state(name, 0);
    }
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
        set_state(name, _gpio.at(name).get_value());
      }
      else if (name.find("dout") != std::string::npos) {
        set_state(name, _gpio.at(name).get_value());
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
        _gpio.at(name).set_value(get_command(name));
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