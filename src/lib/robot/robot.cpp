#include "edu_robot/robot.hpp"
#include "edu_robot/color.hpp"
#include "edu_robot/hardware_error.hpp"
#include "edu_robot/lighting.hpp"

#include "edu_robot/mode.hpp"
#include "edu_robot/msg/detail/mode__struct.hpp"
#include "edu_robot/msg_conversion.hpp"
#include "edu_robot/msg/detail/robot_status_report__struct.hpp"
#include "edu_robot/processing_component/collison_avoidance.hpp"
#include "edu_robot/processing_component/odometry_estimator.hpp"
#include "edu_robot/processing_component/processing_detect_charging.hpp"

#include <Eigen/Dense>
#include <Eigen/QR>

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <memory>
#include <cstdio>
#include <exception>
#include <functional>
#include <stdexcept>
#include <cstddef>

namespace eduart {
namespace robot {

using namespace std::chrono_literals;

static Robot::Parameter get_robot_ros_parameter(rclcpp::Node& ros_node)
{
  Robot::Parameter parameter;
  
  // Declare Parameters
  ros_node.declare_parameter<std::string>("tf_base_frame", parameter.tf_base_frame);
  ros_node.declare_parameter<std::string>("tf_footprint_frame", parameter.tf_footprint_frame);
  ros_node.declare_parameter<bool>("publish_tf_odom", parameter.publish_tf_odom);
  ros_node.declare_parameter<bool>("collision_avoidance.enable", parameter.enable_collision_avoidance);
  ros_node.declare_parameter<float>("collision_avoidance.distance_reduce_velocity", parameter.collision_avoidance.distance_reduce_velocity);
  ros_node.declare_parameter<float>("collision_avoidance.distance_velocity_zero", parameter.collision_avoidance.distance_velocity_zero);

  // Get Parameter Values
  parameter.tf_base_frame = ros_node.get_parameter("tf_base_frame").as_string();
  parameter.tf_footprint_frame = ros_node.get_parameter("tf_footprint_frame").as_string();
  parameter.publish_tf_odom = ros_node.get_parameter("publish_tf_odom").as_bool();
  parameter.enable_collision_avoidance = ros_node.get_parameter("collision_avoidance.enable").as_bool();
  parameter.collision_avoidance.distance_reduce_velocity = ros_node.get_parameter("collision_avoidance.distance_reduce_velocity").as_double();
  parameter.collision_avoidance.distance_velocity_zero = ros_node.get_parameter("collision_avoidance.distance_velocity_zero").as_double();

  return parameter;
}

Robot::Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface, const std::string& ns)
  : rclcpp::Node(robot_name, ns)
  , _hardware_interface(std::move(hardware_interface))
  , _tf_broadcaster(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  _parameter = get_robot_ros_parameter(*this);

  // Publisher
  _pub_odometry = create_publisher<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::QoS(2).reliable().durability_volatile()
  );
  _pub_status_report = create_publisher<edu_robot::msg::RobotStatusReport>(
    "status_report",
    rclcpp::QoS(2).best_effort().durability_volatile()
  );
  _pub_kinematic_description = create_publisher<edu_robot::msg::RobotKinematicDescription>(
    "robot_kinematic_description",
    rclcpp::QoS(2).reliable().transient_local()
  );

  // Services
  _srv_set_mode = create_service<edu_robot::srv::SetMode>(
    "set_mode",
    std::bind(&Robot::callbackServiceSetMode, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Subscriptions
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

  // Initialize State Machine, switch from unconfigured to inactive.
  configureStateMachine();
  _mode_state_machine.switchToMode(RobotMode::INACTIVE);

  // Timers
  _timer_status_report = create_wall_timer(100ms, std::bind(&Robot::processStatusReport, this));
  _timer_tf_publishing = create_wall_timer(100ms, std::bind(&Robot::processTfPublishing, this));
  _timer_watch_dog = create_wall_timer(500ms, std::bind(&Robot::processWatchDogBarking, this));

  // Initialize Processing Components
  _collision_avoidance_component = std::make_shared<processing::CollisionAvoidance>(
    _parameter.collision_avoidance,
    *this
  );
  _detect_charging_component = std::make_shared<processing::DetectCharging>(
    processing::DetectCharging::Parameter{},
    *this
  );
  _odometry_component = std::make_shared<processing::OdometryEstimator>(
    processing::OdometryEstimator::Parameter{},
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

    // Reduce the velocity if collision avoidance was enabled.
    if (_parameter.enable_collision_avoidance
        && (_mode_state_machine.mode().feature_mode & FeatureMode::COLLISION_AVOIDANCE_OVERRIDE) == false) {
      // \todo Move lines blew in collision avoidance processing component.
      if (velocity_cmd.x() > 0.0) {
        // Driving Forward
        velocity_cmd.x() *= _collision_avoidance_component->getVelocityReduceFactorFront();
        velocity_cmd.z() *= _collision_avoidance_component->getVelocityReduceFactorFront();
      }
      else if (velocity_cmd.x() < 0.0) {
        // Driving Backwards
        velocity_cmd.x() *= _collision_avoidance_component->getVelocityReduceFactorRear();
        velocity_cmd.z() *= _collision_avoidance_component->getVelocityReduceFactorRear();
      }
      else {
        // No Driving in x direction, only rotation is addressed.
        const float reduce_factor = std::min(
          _collision_avoidance_component->getVelocityReduceFactorFront(),
          _collision_avoidance_component->getVelocityReduceFactorRear()
        );
        velocity_cmd.z() *= reduce_factor;
      }
    }

    // Calculate wheel rotation speed using provided kinematic matrix.
    // Apply velocity reduction if a limit is reached.
    Eigen::VectorXf radps = _kinematic_matrix * velocity_cmd;
    float reduce_factor = 1.0f;
  
    for (Eigen::Index i = 0; i < radps.size(); ++i) {
      reduce_factor = std::min(
        std::abs(_motor_controllers[i]->parameter().max_rpm / Rpm::fromRadps(radps(i))), reduce_factor
      );
    }
    for (Eigen::Index i = 0; i < radps.size(); ++i) {
      _motor_controllers[i]->setRpm(Rpm::fromRadps(radps(i)) * reduce_factor);
    }

    // Calculating Odometry and Publishing it
    Eigen::VectorXf radps_measured(_motor_controllers.size());

    for (std::size_t i = 0; i < _motor_controllers.size(); ++i) {
      radps_measured(i) = _motor_controllers[i]->getMeasuredRpm().radps();
    }

    const Eigen::Vector3f velocity_measured = _inverse_kinematic_matrix * radps_measured;
    _odometry_component->process(velocity_measured);
    _pub_odometry->publish(_odometry_component->getOdometryMessage(
      getFrameIdPrefix() + _parameter.tf_footprint_frame, getFrameIdPrefix() + "odom"
    ));

    if (_parameter.publish_tf_odom) {
      _tf_broadcaster->sendTransform(_odometry_component->getTfMessage(
        getFrameIdPrefix() + _parameter.tf_footprint_frame, getFrameIdPrefix() + "odom"
      ));
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
    // Robot Mode Request Handling
    if (request->mode.mode != edu_robot::msg::Mode::UNKNOWN) {
      _mode_state_machine.switchToMode(static_cast<RobotMode>(request->mode.mode));
    }
    // Drive Kinematic Request Handling
    if (request->mode.drive_kinematic != edu_robot::msg::Mode::UNKNOWN) {
      switchKinematic(static_cast<DriveKinematic>(request->mode.drive_kinematic));
      _mode_state_machine.setDriveKinematic(static_cast<DriveKinematic>(request->mode.drive_kinematic));
    }
    // Feature Request Handling
    if (request->mode.feature_mode != edu_robot::msg::Mode::NONE) {
      _mode_state_machine.enableFeature(static_cast<FeatureMode>(request->mode.feature_mode));
    }
    if (request->disable_feature != edu_robot::msg::Mode::NONE) {
      _mode_state_machine.disableFeature(static_cast<FeatureMode>(request->disable_feature));
    }

    response->state.info_message = "OK";
    response->state.state.value = response->state.state.OK;
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to set new mode. what() = " << ex.what());
    response->state.info_message = "REJECTED";
    response->state.state.value = response->state.state.OK;        
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to set new mode. what() = " << ex.what());
    response->state.info_message = "REJECTED";
    response->state.state.value = response->state.state.OK;      
  }

  response->state.mode.mode = static_cast<std::uint8_t>(_mode_state_machine.mode().robot_mode);
  response->state.mode.drive_kinematic = static_cast<std::uint8_t>(_mode_state_machine.mode().drive_kinematic);
  response->state.mode.feature_mode = static_cast<std::uint8_t>(_mode_state_machine.mode().feature_mode);  
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
  try {
    auto report = _hardware_interface->getStatusReport();
    _pub_status_report->publish(toRos(report));
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while getting status report. what() = " << ex.what());                                      
    return;
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while getting status report. what() = " << ex.what());
    return;
  }
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
  try {
    static bool last_state = false;

    if (last_state == false && _detect_charging_component->isCharging() == true) {
      _mode_state_machine.switchToMode(RobotMode::CHARGING);
    }

    last_state = _detect_charging_component->isCharging();
  }
  catch (HardwareError& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Hardware error occurred while trying to process timercallback. what() = " << ex.what());                                      
    return;
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error occurred while trying to process timercallback. what() = " << ex.what());
    return;
  }  
}

void Robot::setLightingForMode(const RobotMode mode)
{
  auto search = _lightings.find("all");

  if (search == _lightings.end()) {
    RCLCPP_WARN(get_logger(), "Can't set lighting to indicate inactive mode. Lighting \"all\" was not found.");
    return;
  }

  switch (mode) {
    case RobotMode::UNCONFIGURED:
    case RobotMode::INACTIVE:
      search->second->setColor(Color{34, 0, 0}, Lighting::Mode::PULSATION);
      break;

    case RobotMode::REMOTE_CONTROLLED:
      search->second->setColor(Color{34, 34, 34}, Lighting::Mode::DIM);
      break;

    case RobotMode::FLEET:
      search->second->setColor(Color{25, 25, 25}, Lighting::Mode::FLASH);
      break;

    case RobotMode::CHARGING:
      search->second->setColor(Color{34, 0, 0}, Lighting::Mode::ROTATION);
      break;
    
    default:
      break;
  }
}

void Robot::remapTwistSubscription(const std::string& new_topic_name)
{
  const auto qos = _sub_twist->get_actual_qos();
  _sub_twist = create_subscription<geometry_msgs::msg::Twist>(
    new_topic_name, qos, std::bind(&Robot::callbackVelocity, this, std::placeholders::_1)
  );
}

void Robot::configureStateMachine()
{
  // Remote Control Mode
  _mode_state_machine.setModeActivationOperation(RobotMode::REMOTE_CONTROLLED, [this](){
    _hardware_interface->enable();
    setLightingForMode(RobotMode::REMOTE_CONTROLLED);
  });

  // Inactive Mode
  _mode_state_machine.setModeActivationOperation(RobotMode::INACTIVE, [this](){
    _hardware_interface->disable();
    setLightingForMode(RobotMode::INACTIVE);
  });

  // Fleet Mode
  _mode_state_machine.setModeActivationOperation(RobotMode::FLEET, [this](){
    remapTwistSubscription("fleet/cmd_vel");
    switchKinematic(DriveKinematic::MECANUM_DRIVE);
    _hardware_interface->enable();
    setLightingForMode(RobotMode::FLEET);
  });
  _mode_state_machine.setModeDeactivationOperation(RobotMode::FLEET, [this](){
    remapTwistSubscription("cmd_vel");
  });

  // Charging Mode
  _mode_state_machine.setModeActivationOperation(RobotMode::CHARGING, [this](){
    _hardware_interface->disable();
    setLightingForMode(RobotMode::CHARGING);
  });
}

std::string Robot::getFrameIdPrefix() const
{
  // remove slash at the beginning
  std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  // add slash at the end if it is missing
  if (frame_id_prefix.back() != '/') {
    frame_id_prefix.push_back('/');
  }
  return frame_id_prefix;
}

void Robot::switchKinematic(const DriveKinematic kinematic)
{
  _kinematic_matrix = getKinematicMatrix(kinematic);
  _inverse_kinematic_matrix = _kinematic_matrix.completeOrthogonalDecomposition().pseudoInverse();

  edu_robot::msg::RobotKinematicDescription msg;

  msg.k.cols = _kinematic_matrix.cols();
  msg.k.rows = _kinematic_matrix.rows();
  msg.k.data.resize(_kinematic_matrix.cols() * _kinematic_matrix.rows());

  for (Eigen::Index row = 0; row < _kinematic_matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < _kinematic_matrix.cols(); ++col) {
      msg.k.data[row * _kinematic_matrix.cols() + col] = _kinematic_matrix(row, col);
    }
  }
  for (const auto& motor : _motor_controllers) {
    msg.wheel_limits.push_back(motor.second->parameter().max_rpm);
  }

  _pub_kinematic_description->publish(msg);
}

} // end namespace robot
} // end namespace eduart
