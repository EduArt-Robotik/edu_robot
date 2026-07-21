/**
 * Copyright EduArt Robotik GmbH 2026
 */
#pragma once

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include <sensorring/logger/Logger.hpp>
#include <sensorring/subscription/Subscription.hpp>

#include <atomic>
#include <mutex>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

/**
 * @brief Bridges edu_lib_sensorring log messages to ROS logging.
 *
 * Implemented as a leaking singleton so its RAII subscription stays active
 * for the full process lifetime.
 */
class SensorLogAdapter {
public:
  SensorLogAdapter(const SensorLogAdapter &) = delete;
  SensorLogAdapter &operator=(const SensorLogAdapter &) = delete;

  static SensorLogAdapter *getInstance() noexcept;

  /**
   * @brief Initialize runtime parameter bridge exactly once.
   *
   * Declares and reads sensorring.logging.enabled and installs a dynamic
   * parameter callback to allow enabling/disabling forwarding at runtime.
   */
  void initialize(rclcpp::Node &ros_node);

private:
  SensorLogAdapter();

  sensorring::subscription::Subscription _subscription;
  std::atomic_bool _enabled{true};
  std::once_flag _init_once;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      _parameter_callback_handle;
};

} // namespace can_gateway
} // namespace hardware
} // namespace robot
} // namespace eduart
