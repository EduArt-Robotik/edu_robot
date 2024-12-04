# Changing Middleware

The ROS2 middleware is normally set system-wide for EduArt robots via an environment variable in the /etc/environment file. This variable is then either transferred to our Docker containers or is also valid for natively installed ROS2 applications.

> **__Note:__** This manual refers primarily to edu_robot which is deployed via Docker. If you deploy this application differently and need help, please contact the EduArt Service.

## Selecting Middleware via Environment File

Our robots usually come with pre-installed software including parameters and configuration. This also includes the middleware parameterization. In this case, the middleware should be set via the /etc/environment file.

Open the file with the following command:

```bash
sudo nano /etc/environment
```

In this file you will find the environment variable **RMW_IMPLEMENTATION**.

```bash
# ROS2 Middleware
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Cyclone DDS is assigned by default. To change the middleware, please select one from the following list and replace the current one.

* rmw_fastrtps_cpp (FastRTPS)
* rmw_cyclonedds_cpp (CycloneDDS)

The easiest way to make the setting effective in the system is to reboot the robot.

```bash
sudo reboot
```

After the reboot each Docker container needs to be restarted. In case of edu_robot please do following:

```bash
cd ~/edu_robot/docker/iot2050
docker compose down
docker compose up
```

> **__Note:__** If other EduArt containers are running on the robot, these must also be restarted. 