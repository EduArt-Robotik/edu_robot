/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_robot/motor_controller.hpp"
#include "edu_robot/robot_hardware_interface.hpp"
#include "edu_robot/sensor.hpp"
#include "edu_robot/processing_component/collison_avoidance.hpp"

#include "edu_robot/msg/mode.hpp"
#include "edu_robot/msg/set_lighting_color.hpp"
#include "edu_robot/msg/robot_status_report.hpp"
#include "edu_robot/srv/set_mode.hpp"

#include <Eigen/Core>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <map>
#include <string>

namespace eduart {
namespace robot {

class Lighting;

/**
 * \brief Defines and implements basic functionality of robot in Eduart universe. Each robot inherits from this
 *        class has to implement a hardware driver that is used for hardware abstraction.
 */
class Robot : public rclcpp::Node
{
protected:
  Robot(const std::string& robot_name, std::unique_ptr<RobotHardwareInterface> hardware_interface);

public:
  struct Parameter {
    std::string tf_base_frame = "base_link";
    bool enable_collision_avoidance = false;
  };

  virtual ~Robot();

  /**
   * \brief Callback used to process received twist messages by passing it to the hardware abstraction layer.
   *
   * \param twist_msg Received twist message.
   */
  void callbackVelocity(std::shared_ptr<const geometry_msgs::msg::Twist> twist_msg);
  /**
   * \brief Callback used to set the color and brightness of connected lightings.
   *
   * \param msg Received set lighting color message.
   */
  void callbackSetLightingColor(std::shared_ptr<const edu_robot::msg::SetLightingColor> msg);
  /**
   * \brief Callback to process a set mode request. If the request was accepted a OK state will be responded.
   *        If it wasn't accepted the error state will be responded.
   *
   * \param request The request mode.
   * \param response The response to the send request.
   */
  void callbackServiceSetMode(const std::shared_ptr<edu_robot::srv::SetMode::Request> request,
                              std::shared_ptr<edu_robot::srv::SetMode::Response> response);

protected:
  // Configuration Methods
  void registerLighting(std::shared_ptr<Lighting> lighting);
  void registerMotorController(std::shared_ptr<MotorController> motor_controller);
  void registerSensor(std::shared_ptr<Sensor> sensor);

  inline std::shared_ptr<tf2_ros::TransformBroadcaster> getTfBroadcaster() { return _tf_broadcaster; }

  // Parameter
  Parameter _parameter;

  // Hardware Interface
  std::shared_ptr<RobotHardwareInterface> _hardware_interface;

  // Drive Kinematic
  Eigen::MatrixXf _kinematic_matrix;

  // Processing Components
  std::shared_ptr<processing::CollisionAvoidance> _collision_avoidance_component;

private:
  void processStatusReport();
  void processTfPublishing();

  // ROS related members
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> _pub_odometry;
  std::shared_ptr<rclcpp::Publisher<edu_robot::msg::RobotStatusReport>> _pub_status_report;

  std::shared_ptr<rclcpp::Service<edu_robot::srv::SetMode>> _srv_set_mode;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> _sub_twist;
  std::shared_ptr<rclcpp::Subscription<edu_robot::msg::SetLightingColor>> _sub_set_lighting_color;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  // Timer used for synchronous processing
  std::shared_ptr<rclcpp::TimerBase> _timer_status_report; 
  std::shared_ptr<rclcpp::TimerBase> _timer_tf_publishing;

  // Mounted components that are controlled by the hardware interface.
  std::map<std::string, std::shared_ptr<Lighting>> _lightings;
  std::map<std::uint8_t, std::shared_ptr<MotorController>> _motor_controllers;
  std::map<std::string, std::shared_ptr<Sensor>> _sensors;
};

} // end namespace robot
} // end namespace eduart
