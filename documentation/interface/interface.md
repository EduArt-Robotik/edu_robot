# EduArt's Robot ROS Interfaces

A basic interface to the EduArt robots was defined. This interfaces are valid for all our robots. Following inputs are required for an operating robot:

![](../image/eduart-robot-interfaces.png)

## Topics

| Description                     | Topic                    | Type  | Message Type |
|---------------------------------|--------------------------|-----------|---------------------------|
| Velocity Input                  | /eduard/cmd_vel            | Input | sensor_msgs/msg/Twist                |
| Set Lighting Color/Mode         | /eduard/set_lighting_color | Input | edu_robot/msg/SetLightingColor       |
| TF | /tf | In-/Output | |
| Measured Odometry | /eduard/odom | Output | nav_msgs/Odometry |
| Robot Status Report            | /eduard/status_report | Output | edu_robot/msg/RobotStatusReport      |
| Join States of the Wheels       | /joint_states | Output            | sensor_msgs/msg/JointState           |

Note: "/eduard" in topic name is the default namespace if no other was defined. This namespace can be freely defined by the "namespace" ROS parameter. Please take into account that the topic name changes according to the namespace.

## Services

| Description                     | Service                  | Message type                         |
|---------------------------------|--------------------------|--------------------------------------|
| Set Mode Service (used for Enable robot) | /eduard/set_mode       | edu_robot/srv/SetMode                |

Note: "/eduard" in service name is the default namespace if no other was defined. This namespace can be freely defined. Please take into account that the service name changes according to the namespace, too.

## Different Operation Modes

The minium input the robots require are the velocity command and the service "set_mode". Without an velocity command, the robot will detect an timeout and switch of the motor controller. This leads in an "inactive" robot (disabled). If the velocity input is send the robot can be activated for remote control drive by calling the service "set_mode". Below all current implemented modes are listed:

| Mode | Description |
|------|-------------|
| INACTIVE | The robot is inactive. All drives are disabled. The velocity commands have no effect. |
| REMOTE_CONTROLLED | The robot is active. All drives are enabled. The robot processes the velocity commands. |
| AUTONOMOUS | The robot is active. The mecanum drive kinematic is used. The robot will subscribe to the topic "autonomous/cmd_vel". |
| SKID_DRIVE | Uses the kinematic of an skid drive. Note: only available if the robot supports it. |
| MECANUM_DRIVE | Uses the kinematic of an mecanum drive. Note: only available if the robot supports it. |
| COLLISION_AVOIDANCE_OVERRIDE_ENABLED | If the robots accepts it the integrated collision avoidance will be overridden. |
| COLLISION_AVOIDANCE_OVERRIDE_DISABLED | If the robot accepts it the integrated collision avoidance will be activated when it is enabled in general. |

These modes are combinable, but they must be requested separately by the "set_mode" service. In the response of this service a list of the robot's complete mode is sent.

Note: a ready to use package is available for controlling the robot by an Gamepad or Joystick. Please see section "Controlling the Robot" or visit the repository [edu_robot_control](https://github.com/EduArt-Robotik/edu_robot_control).

# Robot Eduard ROS Interface

Eduard is our main robot system. It consists from of four wheels, four lightings including range sensors. And of an integrated IMU sensor. The robot realizes the above defined interfaces and expand it by the following ones:

| Description                     | Topic                    | Message type                         |
|---------------------------------|--------------------------|--------------------------------------|
| Range Sensor Output Front Left  | /range/front/left/range  | sensor_msgs/msg/Range                |
| Range Sensor Output Front Right | /range/front/right/range | sensor_msgs/msg/Range                |
| Range Sensor Output Rear Left   | /range/rear/left/range   | sensor_msgs/msg/Range                |
| Range Sensor Output Rear Right  | /range/rear/right/range  | sensor_msgs/msg/Range                |
