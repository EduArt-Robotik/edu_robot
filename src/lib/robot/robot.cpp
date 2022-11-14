#include "edu_robot/robot.hpp"
#include "edu_robot/hardware_error.hpp"
#include "edu_robot/lighting.hpp"

#include "edu_robot/mode.hpp"
#include "edu_robot/msg/detail/mode__struct.hpp"
#include "edu_robot/msg_conversion.hpp"
#include "edu_robot/msg/detail/robot_status_report__struct.hpp"
#include "edu_robot/processing_component/collison_avoidance.hpp"
#include "edu_robot/processing_component/processing_detect_charging.hpp"

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>

#include <cstdio>
#include <exception>
#include <functional>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <stdexcept>

namespace eduart {
namespace robot {

using namespace std::chrono_literals;

static Robot::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  Robot::Parameter parameter;
  
  // Declare Parameters
  ros_node.declare_parameter<std::string>("tf_base_frame", parameter.tf_base_frame);
  ros_node.declare_parameter<bool>("enable_collision_avoidance", parameter.enable_collision_avoidance);

  // Get Parameter Values
  parameter.tf_base_frame = ros_node.get_parameter("tf_base_frame").as_string();
  parameter.enable_collision_avoidance = ros_node.get_parameter("enable_collision_avoidance").as_bool();

  return parameter;
}

Robot::Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface)
  : rclcpp::Node(robot_name)
  , _hardware_interface(std::move(hardware_interface))
  , _mode(Mode::UNCONFIGURED)
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  _parameter = get_robot_ros_parameter(*this);

  _pub_odometry = create_publisher<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::QoS(2).reliable().durability_volatile()
  );
  _pub_status_report = create_publisher<edu_robot::msg::RobotStatusReport>(
    "status_report",
    rclcpp::QoS(2).best_effort().durability_volatile()
  );

  _srv_set_mode = create_service<edu_robot::srv::SetMode>(
    "set_mode",
    std::bind(&Robot::callbackServiceSetMode, this, std::placeholders::_1, std::placeholders::_2)
  );

  _sub_twist = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::QoS(2).best_effort().durability_volatile(),
    std::bind(&Robot::callbackVelocity, this, std::placeholders::_1)
  );
  _sub_set_lighting_color = create_subscription<edu_robot::msg::SetLightingColor>(
    "set_lighting_color",
    rclcpp::QoS(2).best_effort(),
    std::bind(&Robot::callbackSetLightingColor, this, std::placeholders::_1)
  );

  _timer_status_report = create_wall_timer(100ms, std::bind(&Robot::processStatusReport, this));
  _timer_tf_publishing = create_wall_timer(100ms, std::bind(&Robot::processTfPublishing, this));
  _timer_watch_dog = create_wall_timer(500ms, std::bind(&Robot::processWatchDogBarking, this));

  // Initialize Processing Components
  _collision_avoidance_component = std::make_shared<processing::CollisionAvoidance>(
    processing::CollisionAvoidance::Parameter{ 0.3f, 0.05f },
    *this
  );
  _detect_charging_component = std::make_shared<processing::DetectCharging>(
    processing::DetectCharging::Parameter{},
    *this
  );
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
  // Kick Watch Dog
  // _timer_status_report->reset();

  try {
    // \todo maybe a size check would be great!
    Eigen::Vector3f velocity_cmd(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
    // velocity_cmd *= _collision_avoidance_component->getVelocityReduceFactor();
    Eigen::VectorXf rps = _kinematic_matrix * velocity_cmd / (2.0f * M_PI);

    for (Eigen::Index i = 0; i < rps.size(); ++i) {
      _motor_controllers[i]->setRpm(Rpm::fromRps(rps(i)));
    }
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to set new values for motor controller."
                                      << " what() = " << ex.what());                                      
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to set new values for motor controller."
                                      << " what() = " << ex.what());     
  }
}

void Robot::callbackSetLightingColor(std::shared_ptr<const edu_robot::msg::SetLightingColor> msg)
{
  // Kick Watch Dog
  // _timer_status_report->reset();

  const auto search = _lightings.find(msg->lighting_name);

  if (search == _lightings.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Lighting with name \"" << msg->lighting_name
                        << "\" wasn't found. Can't set color.");
    return;                        
  }
  if (_detect_charging_component->isCharging()) {
    // do not accept lighting command during charging
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
  // Kick Watch Dog
  // _timer_status_report->reset();

  try {
    if (request->mode.value == edu_robot::msg::Mode::REMOTE_CONTROLLED) {
      _hardware_interface->enable();
      _mode = Mode::REMOTE_CONTROLLED;
      if (_detect_charging_component->isCharging() == false) {
        setLightingForMode(_mode);
      }
    }
    else if (request->mode.value == edu_robot::msg::Mode::INACTIVE) {
      _hardware_interface->disable();
      _mode = Mode::INACTIVE;
      if (_detect_charging_component->isCharging() == false) {
        setLightingForMode(_mode);
      }
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
  _hardware_interface->clearStatusReport();
}

void Robot::processTfPublishing()
{
  const auto stamp = get_clock()->now();

  for (const auto& sensor : _sensors) {
    _tf_broadcaster->sendTransform(sensor.second->getTransformMsg(stamp));
  }
}

void Robot::processWatchDogBarking()
{
  // HACK: only used for indication of charging at the moment
  // _mode = Mode::INACTIVE;
  // setLightingForMode(_mode);
  static bool last_state = _detect_charging_component->isCharging();

  if (last_state == false && _detect_charging_component->isCharging() == true) {
    setLightingForMode(Mode::INACTIVE);
  }

  last_state = _detect_charging_component->isCharging();
}

void Robot::setLightingForMode(const Mode mode)
{
  auto search = _lightings.find("all");

  if (search == _lightings.end()) {
    RCLCPP_WARN(get_logger(), "Can't set lighting to indicate inactive mode. Lighting \"all\" was not found.");
  }

  if (_detect_charging_component->isCharging()) {
    search->second->setColor(Color{170, 0, 0}, Lighting::Mode::ROTATION);
  }
  else {
    switch (mode) {
      case Mode::INACTIVE:
        search->second->setColor(Color{170, 0, 0}, Lighting::Mode::PULSATION);
        break;

      case Mode::REMOTE_CONTROLLED:
        search->second->setColor(Color{170, 170, 170}, Lighting::Mode::DIM);
        break;

      default:
        break;
    }
  }
}

} // end namespace robot
} // end namespace eduart
