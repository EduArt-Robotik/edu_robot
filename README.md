# edu_robot

Welcome to the alpha of EduArt ROS2 robot control software. In first place it provides a ROS2 node for controlling and monitoring the EduArt robots, at the moment Eduard as offroad (skid drive) is supported only. Mecanum kinematic will be supported soon. In future also a ROS2 Gazebo plugin is provided for an easy SIL test environment.

> **_NOTE:_** For further help please visit our [Discord channel](https://discord.gg/tXnjH2cF) or write a e-mail at info@eduart-robotik.com.


# Eduard's ROS Interfaces

Eduard's control node provides following interfaces via ROS topics and services:

| Description                     | Topic                    | Message type                         |
|---------------------------------|--------------------------|--------------------------------------|
| Velocity Input                  | /cmd_vel                 | sensor_msgs/msg/Twist                |
| Set Lighting Color/Mode         | /set_lighting_color      | edu_robot/msg/SetLightingColor       |
| Range Sensor Output Front Left  | /range/front/left/range  | sensor_msgs/msg/Range                |
| Range Sensor Output Front Right | /range/front/right/range | sensor_msgs/msg/Range                |
| Range Sensor Output Rear Left   | /range/rear/left/range   | sensor_msgs/msg/Range                |
| Range Sensor Output Rear Right  | /range/rear/right/range  | sensor_msgs/msg/Range                |
| Odometry Output of Fused Ego Sensors (DO NOT WORK AT THE MOMENT!) | /odometry | nav_msgs/msg/Odometry |
| Status Report Output            | /status_report           | edu_robot/msg/RobotStatusReport      |
| TF Transforms from Sensors      | /tf                      | tf2_msgs/msg/TFMessage               |
| Join States of the Wheels       | /joint_states            | sensor_msgs/msg/JointState           |


| Description                     | Service                  | Message type                         |
|---------------------------------|--------------------------|--------------------------------------|
| Set Mode Service (used for Enable robot) | /set_mode       | edu_robot/srv/SetMode                |

# Deploying on IoT2050

This section describes how the software is deployed on an IoT2050 in a Docker environment. First clone the repository on the robot by executing this command:

```bash
git clone https://github.com/EduArt-Robotik/edu_robot.git
```

Then navigate into the docker folder in the cloned repository:

```bash
cd edu_robot/docker
```

Using make the Docker image can be build:

```bash
make all
make clean
```

After executing these command a new Docker image with the name "eduard-iotbot:alpha" should be created. It can be checked by following command:

```bash
docker image ls
```

The docker container can easily started by the command:

```bash
docker run --name eduart-iotbot-alpha --restart=always --privileged -v /dev:/dev --network host --group-add dialout eduard-iotbot:alpha
```

With the flag "--restart=always" the container will come up after rebooting the system. If this is not wanted please remove this flag.

# Monitoring Eduard using RViz

For visualization of Eduard's sensors and actors a RViz setup is provided including a robot description. Since Eduard ROS control node publish all of Eduard's state via TF and ROS topic/services it is easy to use them.

