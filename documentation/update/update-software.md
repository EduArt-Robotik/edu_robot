# Updating EduArt's ROS2 Software

## Changing ROS2 Middleware

ONLY DO IF NEEDED! Since version 0.5.0 it is possible to select the ROS2 middleware if edu_robot is used in a Docker container. If you want to configure it, you can do so by following [these instructions](changing-middleware.md).

## Checking

To be always up to date, it is worth checking the system regularly for updates. To do this, change to the "home" directory and then into the folder "edu_nodered_ros2_plugin" and execute the commands:

```bash
cd ~/edu_robot
git checkout main
git pull origin
```
The output will tell you if there is a new version of the code. In case 'already up to date' is printed there is no update available.

## Update

When a new version is available it can be launched by executing following command:

```bash
cd ~/edu_robot/docker/iot2050
docker compose up
```

The current executed container will be stopped and the new will be pulled and started. Thats it!

> **Note**: if you leave the terminal using CTRL+C it could be necessary to reboot the robot, because the container stopped temporary.