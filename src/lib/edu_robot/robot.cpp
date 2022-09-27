#include "edu_robot/robot.hpp"
#include "edu_robot/hardware_error.hpp"
#include "edu_robot/lighting.hpp"
#include "edu_robot/msg/detail/mode__struct.hpp"
#include "edu_robot/msg_conversion.hpp"

#include "edu_robot/msg/detail/robot_status_report__struct.hpp"
#include "edu_robot/msg/detail/set_lighting_color__struct.hpp"
#include "edu_robot/srv/detail/set_mode__struct.hpp"
#include <cstdio>
#include <exception>
#include <functional>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {

using namespace std::chrono_literals;

Robot::Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
  : rclcpp::Node(robot_name)
  , _hardware_interface(std::move(hardware_interface))
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  _pub_odometry = create_publisher<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::QoS(2).reliable()
  );
  _pub_status_report = create_publisher<edu_robot::msg::RobotStatusReport>(
    "status_report",
    rclcpp::QoS(2).best_effort()
  );

  _srv_set_mode = create_service<edu_robot::srv::SetMode>(
    "set_mode",
    std::bind(&Robot::callbackServiceSetMode, this, std::placeholders::_1, std::placeholders::_2)
  );

  _sub_twist = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::QoS(2),
    std::bind(&Robot::callbackVelocity, this, std::placeholders::_1)
  );
  _sub_set_lighting_color = create_subscription<edu_robot::msg::SetLightingColor>(
    "set_lighting_color",
    rclcpp::QoS(2).best_effort(),
    std::bind(&Robot::callbackSetLightingColor, this, std::placeholders::_1)
  );

  _timer_status_report = create_wall_timer(100ms, std::bind(&Robot::processStatusReport, this));
  _timer_tf_publishing = create_wall_timer(100ms, std::bind(&Robot::processTfPublishing, this));
}

Robot::~Robot()
{
  // Switch off all lights and set color white.
  for (auto& lighting : _lightings) {
    lighting.second->setColor({0xff, 0xff, 0xff}, Lighting::Mode::OFF);
    lighting.second->setBrightness(0.0f);
  }
}

void Robot::callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg)
{
  // BEGIN HACK
  constexpr float wheel_diameter = 0.17f;
  constexpr float wheel_base = 0.3f;
  constexpr float track = 0.3;

  constexpr float ms2rpm = 60.0f / (wheel_diameter * M_PI);
  constexpr float rad2rpm = (wheel_base + track) / wheel_diameter;

  const float rpmFwd   = twist_msg->linear.x  * ms2rpm;
  const float rpmLeft  = twist_msg->linear.y  * ms2rpm;
  const float rpmOmega = twist_msg->angular.z * rad2rpm;

  _motor_controllers[0u]->setRpm(Rpm( rpmFwd - rpmLeft - rpmOmega));
  _motor_controllers[1u]->setRpm(Rpm(-rpmFwd - rpmLeft - rpmOmega));
  _motor_controllers[2u]->setRpm(Rpm( rpmFwd + rpmLeft - rpmOmega));
  _motor_controllers[3u]->setRpm(Rpm(-rpmFwd + rpmLeft - rpmOmega));
  // END HACK
}

void Robot::callbackSetLightingColor(std::shared_ptr<const edu_robot::msg::SetLightingColor> msg)
{
  const auto search = _lightings.find(msg->lighting_name);

  if (search == _lightings.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Lighting with name \"" << msg->lighting_name
                        << "\" wasn't found. Can't set color.");
    return;                        
  }

  // try to set new values
  try {
    search->second->setColor({ msg->r, msg->g, msg->b }, fromRos(*msg));
    search->second->setBrightness(msg->brightness.data);
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to set new values for lighting \""
                                      << msg->lighting_name << "\". what() = " << ex.what());                                      
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to set new values for lighting \""
                                      << msg->lighting_name << "\". what() = " << ex.what());     
  }
}

void Robot::callbackServiceSetMode(const std::shared_ptr<edu_robot::srv::SetMode::Request> request,
                                   std::shared_ptr<edu_robot::srv::SetMode::Response> response)
{
  try {
    if (request->mode.value == edu_robot::msg::Mode::REMOTE_CONTROLLED) {
      _hardware_interface->enable();
    }
    else if (request->mode.value == edu_robot::msg::Mode::INACTIVE) {
      _hardware_interface->disable();
    }
    else {
      RCLCPP_ERROR_STREAM(get_logger(), "Unsupported mode. Can't set new mode. Canceling.");
      return;
    }

    response->state.mode = request->mode;
    response->state.info_message = "OK";
    response->state.state.value = response->state.state.OK;
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to set new mode. what() = " << ex.what());                                      
    return;
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to set new mode. what() = " << ex.what());
    return;
  }
}

void Robot::registerLighting(std::shared_ptr<Lighting> lighting)
{
  const auto search = _lightings.find(lighting->name());

  if (search != _lightings.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Lighting \"" << lighting->name() << "\" already contained in lighting register."
                                      << " Can't add it twice.");
    return;                                      
  }

  _lightings[lighting->name()] = lighting;
}

void Robot::registerMotorController(std::shared_ptr<MotorController> motor_controller)
{
  const auto search = _motor_controllers.find(motor_controller->id());

  if (search != _motor_controllers.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Motor controller \"" << motor_controller->name() << "\" already contained"
                                      << " in lighting register. Can't add it twice.");
    return;      
  }

  _motor_controllers[motor_controller->id()] = motor_controller;
}

void Robot::registerSensor(std::shared_ptr<Sensor> sensor)
{
  const auto search = _sensors.find(sensor->name());

  if (search != _sensors.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Sensor \"" << sensor->name() << "\" already contained"
                                      << " in sensor register. Can't add it twice.");
    return;                                      
  }

  _sensors[sensor->name()] = sensor;
}

void Robot::processStatusReport()
{
  if (false == _hardware_interface->isStatusReportReady()) {
    return;
  }

  auto report = _hardware_interface->getStatusReport();

  _pub_status_report->publish(toRos(report));
}

void Robot::processTfPublishing()
{
  const auto stamp = get_clock()->now();

  for (const auto& sensor : _sensors) {
    _tf_broadcaster->sendTransform(sensor.second->getTransformMsg(stamp));
  }
}


} // end namespace robot
} // end namespace eduart
