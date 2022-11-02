# edu_robot

Welcome to the alpha of EduArt ROS2 robot control software. In first place it provides a ROS2 node for controlling and monitoring the EduArt robots, at the moment Eduard as offroad (skid drive) is supported only. Mecanum kinematic will be supported soon. In future also a ROS2 Gazebo plugin is provided for an easy SIL test environment.

> **_NOTE:_** For further help please visit our [Discord channel](https://discord.gg/tXnjH2cF) or write a e-mail at info@eduart-robotik.com.


# Eduard's ROS Interfaces
# Deploying on IoT2050

This section describes how the software is deployed on an IoT2050 in a Docker environment.

# Monitoring Eduard using RViz

For visualization of Eduard's sensors and actors a RViz setup is provided including a robot description. Since Eduard ROS control node publish all of Eduard's state via TF and ROS topic/services it is easy to use them.

