#include "edu_robot/hardware/iot_shield/ros2_hardware_adapter.hpp"

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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2HardwareAdapter::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ros2HardwareAdapter::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros2HardwareAdapter::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  return hardware_interface::return_type::OK;
}

} // end namespace iot_shield
} // end namespace hardware
} // end namespace eduart
} // end namespace robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(eduart::robot::hardware::iot_shield::Ros2HardwareAdapter, hardware_interface::SystemInterface)