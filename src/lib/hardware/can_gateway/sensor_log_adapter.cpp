/**
 * Copyright EduArt Robotik GmbH 2026
 */
#include "edu_robot/hardware/can_gateway/sensor_log_adapter.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>

#include <string>
#include <vector>

namespace eduart {
namespace robot {
namespace hardware {
namespace can_gateway {

SensorLogAdapter *SensorLogAdapter::getInstance() noexcept {
  // Intentional leak: keep the subscription alive until process exit.
  static SensorLogAdapter *instance = new SensorLogAdapter;
  return instance;
}

void SensorLogAdapter::initialize(rclcpp::Node &ros_node) {
  std::call_once(_init_once, [this, &ros_node]() {
    constexpr const char *kParamName = "sensorring.logging.enabled";

    ros_node.declare_parameter<bool>(kParamName, false);
    _enabled.store(ros_node.get_parameter(kParamName).as_bool(),
                   std::memory_order_relaxed);

    _parameter_callback_handle = ros_node.add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          for (const auto &parameter : parameters) {
            if (parameter.get_name() == "sensorring.logging.enabled") {
              if (parameter.get_type() != rclcpp::PARAMETER_BOOL) {
                result.successful = false;
                result.reason =
                    "sensorring.logging.enabled must be a bool parameter";
                return result;
              }
              _enabled.store(parameter.as_bool(), std::memory_order_relaxed);
            }
          }

          return result;
        });
  });
}

SensorLogAdapter::SensorLogAdapter()
    : _subscription(sensorring::logger::Logger::getInstance()->subscribe(
          [this](const sensorring::logger::LogVerbosity verbosity,
                 const std::string &msg) {
            if (!_enabled.load(std::memory_order_relaxed)) {
              return;
            }

            const auto &ros_logger = rclcpp::get_logger("SensorRing");
            switch (verbosity) {
            case sensorring::logger::LogVerbosity::Debug:
              RCLCPP_DEBUG(ros_logger, "%s", msg.c_str());
              break;
            case sensorring::logger::LogVerbosity::Info:
              RCLCPP_INFO(ros_logger, "%s", msg.c_str());
              break;
            case sensorring::logger::LogVerbosity::Warning:
              RCLCPP_WARN(ros_logger, "%s", msg.c_str());
              break;
            case sensorring::logger::LogVerbosity::Error:
            case sensorring::logger::LogVerbosity::Exception:
              RCLCPP_ERROR(ros_logger, "%s", msg.c_str());
              break;
            }
          })) {}

} // end namespace can_gateway
} // end namespace hardware
} // end namespace robot
} // end namespace eduart
