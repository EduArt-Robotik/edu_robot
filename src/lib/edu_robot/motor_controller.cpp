#include "edu_robot/motor_controller.hpp"
#include "edu_robot/rotation_per_minute.hpp"
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

namespace eduart {
namespace robot {

MotorController::MotorController(const std::string& name, const std::uint8_t id, const Parameter& parameter,
                                 const std::string& urdf_joint_name, rclcpp::Node& ros_node)
  : _parameter(parameter)
  , _name(name)
  , _id(id)
  , _urdf_joint_name(urdf_joint_name)
  , _clock(ros_node.get_clock())
  , _stamp_last_measurement(_clock->now())
  , _current_wheel_position(0.0)
{
  _pub_joint_state = ros_node.create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::QoS(2).best_effort().transient_local()
  );
}

MotorController::~MotorController()
{

}

void MotorController::setRpm(const Rpm rpm)
{
  processSetRpm(rpm);
  _set_rpm = rpm;
}

void MotorController::processMeasurementData(const Rpm rpm)
{
  // \todo Check if calculation is correct! At the moment used for visualization only, so no need for accurate calc...
  // perform wheel position calculation
  const auto stamp = _clock->now();
  const auto delta_t = stamp - _stamp_last_measurement;

  _current_wheel_position += delta_t.seconds() * rpm.rps();
  _measured_rpm = rpm;
  _stamp_last_measurement = stamp;

  // publish wheel position as tf message
  // geometry_msgs::msg::TransformStamped msg;

  // publish wheel speed using joint state message
  sensor_msgs::msg::JointState joint_state_msg;

  joint_state_msg.header.frame_id = name(); // motor controller has no frame id --> using its name...
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.name.push_back(_urdf_joint_name);
  joint_state_msg.velocity.push_back(rpm.radps());

  _pub_joint_state->publish(joint_state_msg);
}

} // end namespace robot
} // end namespace eduart
