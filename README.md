# edu_robot - Control Software for IoT Shield and Ethernet Gateway

Welcome to the EduArt ROS2 robot control software. In first place it provides a ROS2 node for controlling and monitoring the EduArt robots, most likely the robot Eduard in different hardware realizations like the IoT Bot or IPC Bot. There is also a [digital twin](https://github.com/EduArt-Robotik/edu_simulation) in the making, which is not yet feature complete but already the basic interface implements.

This packages also designed to used in a robot fleet setup. Basically it is able to handle multiple robot instances at the same time using namespaces.

> **_NOTE:_** For further help please visit our [Forum](https://forum.eduart-robotik.com) or write a e-mail at info@eduart-robotik.com.

# EduArt's Robot - Eduard

Eduard comes in two hardware realizations. Both using this control software. Below you will find documentation about how to install, update, control and monitor the robot.

![](documentation/image/eduard-orange.jpg)

## ROS2 Interfaces

The software package 'edu_robot' provides ROS2 interface. For details please follow the link below:

[ROS2 Interfaces](documentation/interface/interface.md)

## Lighting Codes

| Mode | Color | Description |
|------|-------|-------------|
|UNCONFIGURED| white (pulsation) | During boot up. |
|INACTIVE| white (dim) | Motors are disabled, no error state present. First state after reboot. |
|REMOTE_CONTROLLED| white (dim) | Motors are enabled, robot is ready for driving remote controlled. |
|AUTONOMOUS| white (flashing) | Robot is in fleet mode and enabled. |
|CHARGING| green (pulsation) | |


## Deploying

### On IoT2050

* [First Time Setup](documentation/setup/iot2050/setup_iot2050.md)
* [Update ROS Software](documentation/update/update-software.md)
* [Setup PS5 Controller](documentation/setup/joystick/ps5-gamepad.md)

### On IPC127e

* [First Time Setup](documentation/setup/ipc127e/setup_ipc127.md)
* [Update ROS Software](documentation/update/update-software.md)
* [Setup PS5 Controller](documentation/setup/joystick/ps5-gamepad.md)

### As Siemens Industrial Application

> **Note**: will be updated soon

# Controlling the Robot

## Controller

With the package [edu_robot_control](https://github.com/EduArt-Robotik/edu_robot_control) the EduArt's robots can be controlled remotely. The basic information about how to set up the joystick is also listed below. 

> **Note**: the package "edu_robot_control" don't need to be deployed extra. It is included in the "edu_robot" deployment.

A controller can be requested to connect by pressing a specific button once. For the recommended controllers, it is the symbol between the axes. To operate the Robot, the following buttons and axes of the controller are assigned as follows:


| Axis  | DS5                       | Idle position | Value range | function          | 
|-------|---------------------------|---------------|-------------|-------------------|
| [0]   | Joystick L: left & right  | 0.0           | 1.0 to -1.0 | Moving Forward/Backwards
| [1]   | Joystick L: up & down     | 0.0           | 1.0 to -1.0 | Moving Sidewards
| [2]   | L2                        | 1.0           | 1.0 to -1.0 | not in use
| [3]   | Joystick R: left & right  | 0.0           | 1.0 to -1.0 | Steering
| [4]   | Joystick R: up & down     | 0.0           | 1.0 to -1.0 | not in use
| [5]   | R2                        | 1.0           | 1.0 to -1.0 | not in use
| [6]   | D-Pad: left & right       | 0.0           | 1.0 to -1.0 | not in use
| [7]   | D-Pad: up & down          | 0.0           | 1.0 to -1.0 | not in use

| Button    | DS5           | Idle position | Value range   | function          | 
|-----------|---------------|---------------|---------------|-------------------|
| [0]       | Square        | 0             | 0 or 1        | Switch to Skid Drive Kinematic
| [1]       | Cross         | 0             | 0 or 1        | Light pattern: Operation
| [2]       | Circle        | 0             | 0 or 1        | Switch to Mecanum Drive Kinematic
| [3]       | Triangle      | 0             | 0 or 1        | Light pattern: Operation
| [4]       | L1            | 0             | 0 or 1        | Light pattern: Turning left
| [5]       | R1            | 0             | 0 or 1        | Light pattern: Turning right
| [6]       | L2            | 0             | 0 or 1        | Enable Fleet Drive
| [7]       | R2            | 0             | 0 or 1        | Override collision avoidance
| [8]       | SHARE         | 0             | 0 or 1        | Disable driving
| [9]       | OPTIONS       | 0             | 0 or 1        | Enable driving
| [10]      | PS            | 0             | 0 or 1        | Connect Controller to Eduard
| [11]      | L3            | 0             | 0 or 1        | not in use
| [12]      | R3            | 0             | 0 or 1        | not in use
| [13]      | Map           | 0             | 0 or 1        | Light pattern: Warning light


## Node Red

We have a Node Red Web Server, which can also be deployed as a Docker container. With this it is possible to use Topic, Services and Actions to control the robot. For more information please use the following link:

[Node Red Web Server](https://github.com/EduArt-Robotik/edu_nodered_ros2_plugin)

# Monitoring Eduard using ROS Tools

## RViz2

With the package [edu_robot_control](https://github.com/EduArt-Robotik/edu_robot_control) the EduArt's robots can be monitored using the tool RViz2 coming with ROS2.

For visualization of Eduard's sensors and actors a RViz setup is provided including a robot description. Since Eduard ROS control node publish all of Eduard's states via TF and ROS topic/services it is easy to access them.

The best way to monitor Eduard's states is using RViz. In the package "edu_robot_control" a launch file is provided including a RViz configuration that allows an easy and fast start. Two ways are supported.

### Native ROS2 Installation

If ROS is natively installed the "edu_robot_control" package can be installed into an ROS workspace. As first step clone the package into the workspace by:

```bash
git clone https://github.com/EduArt-Robotik/edu_robot_control.git
```

Please make sure the package will be cloned into the "src" folder in the workspace. If no knowledge about ROS is present please see [docs.ros.org](https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) for further information. 

After the package was cloned it needs to be installed via:

```bash
colcon build --packages-select edu_robot_control --symlink-install
```

Now RViz with the correct configuration can be launched by:

```bash
ros2 launch edu_robot_control eduard_monitor.launch.py
```

If RViz comes up properly it will be shown following:

![Eduard visualized using RViz](documentation/image/eduard-red-rviz.png)

#### Important: Setting Correct Namespace

Each Eduard robot comes with a preset namespace, e.g. to reflect the robot's color. This allows that multiple robots are connected to the same network (with same DOMAIN_ID). However this makes it necessary to deal with the namespace, too, when the robot's data shall be received. For example when displaying data using RViz2.

> Note: when a namespace is used the **Fixed Frame** has to be reselected, otherwise the robot is not displayed correctly.

The namespace can be set by using an environment variable. Either set it via the system or define it in front of the ros command:

```bash
EDU_ROBOT_NAMESPACE=eduard/red ros2 launch edu_robot_control eduard_monitor.launch.py
```

This this case it was set to **eduard/red**. Please replace the color accordingly to your robot. RViz will respect the color and will coloring the robot model. At the moment three colors are available:

![eduard-red](documentation/image/eduard-red.png)![eduard-green](documentation/image/eduard-green.png)![eduard-blue](documentation/image/eduard-blue.png)

#### Select Wheel Type

Eduard comes with two types of wheel. If wanted it can be selected in RViz for a correct visualization by setting a environment variable in front of the ros command:

```bash
EDU_ROBOT_WHEEL_TYPE=offroad ros2 launch edu_robot_control eduard_monitor.launch.py
```

The following two wheel types are available: mecanum and offroad

![eduard-mecanum](documentation/image/eduard-red-mecanum.png)![eduard-offroad](documentation/image/eduard-red-offroad.png)

## rqt robot monitor

The Eduard control software provides a diagnostic aggregation. With this, the status of major components can be displayed, as well as live characteristics of these.

With the standard tool **rqt_robot_monitor** this diagnostic aggregation can be displayed. The application can be started by:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

Following window will open. Errors and warnings will be shown on the both top lists. By double click on an entry a more detailed windows will open.

![error-case](documentation/image/diagnostic-error-case-2.png)

 If you want to see the **OK** states, too, then press on the check box **Alternative view** on the top left corner.

 ![good-case](documentation/image/diagnostic-good-case.png)