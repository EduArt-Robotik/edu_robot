#include "edu_robot/hardware/iot_shield/ros2_hardware_adapter.hpp"

#include <iostream>
#include <mraa/gpio.hpp>

namespace eduart {
namespace robot {
namespace hardware {
namespace iot_shield {

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::SystemInterface::on_init(info);
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);



  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_configure(const rclcpp_lifecycle::State & previous_state)
{
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
          auto pwm = std::make_shared<mraa::Pwm>(pin);
          pwm->period_us(period_us);
          pwm->enable(true);
          pwm->write(0.0f);
          _pwm[gpio.name + '/' + command.name] = pwm;
        }
        else if (command.name.find("dout") != std::string::npos) {
          // digital output pin
          const int pin = std::stoi(command.parameters.at("pin"));
          auto output = std::make_shared<mraa::Gpio>(pin);
          output->dir(mraa::DIR_OUT);
          output->write(0);
          _gpio[gpio.name + '/' + command.name] = output;
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
          auto input = std::make_shared<mraa::Gpio>(pin);
          input->dir(mraa::DIR_IN);
          _gpio[gpio.name + '/' + state.name] = input;
        }
        else if (state.name.find("analog") != std::string::npos) {
          // analog input
          const int pin = std::stoi(state.parameters.at("pin"));
          auto input = std::make_shared<mraa::Aio>(pin);
          _aio[gpio.name + '/' + state.name] = input;
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
  std::cout << "analog:\n";
  for (const auto& [name, egal] : _aio) {
    std::cout << name << std::endl;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "activating hardware and set it to default value");

  // Setting Output Default Values, also Reflecting back State
  for (auto& [name, gpio] : _gpio) {
    if (gpio->readDir() == mraa::DIR_OUT) { 
      gpio->write(0);
      set_command(name, 0);
      set_state(name, 0);
    }
  }
  for (auto& [name, pwm] : _pwm) {
    pwm->write(0.0);
    pwm->enable(true);
    set_command(name, 0.0);
    set_state(name, 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
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
        set_state(name, _gpio.at(name)->read());
      }
      else if (name.find("analog") != std::string::npos) {
        // analog input
        set_state(name, _aio.at(name)->readFloat());
      }
      else if (name.find("pwm") != std::string::npos) {
        set_state(name, get_command(name));
      }
      else if (name.find("dout") != std::string::npos) {
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
  // RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  (void)time;
  (void)period;

  for (const auto& [name, description] : gpio_command_interfaces_) {
    try {
      if (name.find("dout") != std::string::npos) {
        // digital output
        _gpio.at(name)->write(get_command(name));
      }
      else if (name.find("pwm") != std::string::npos) {
        // pwm output
        _pwm.at(name)->write(get_command(name));
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

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(eduart::robot::hardware::iot_shield::Ros2HardwareAdapter, hardware_interface::SystemInterface)