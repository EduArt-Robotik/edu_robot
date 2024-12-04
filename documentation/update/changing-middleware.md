# Changing Middleware

ROS2 uses the environment variable `RMW_IMPLEMENTATION` to adjust which middleware to use. If its empty/non existent, the default (FastRTPS) is active. The EduArt robots set this variable based on the content of an file (`/etc/environment`). This variable will then be set inside each container. 

> **__Note:__** This manual refers primarily to edu_robot which is deployed via Docker. If you deploy this application differently and need help, please contact the EduArt Service.

## Selecting Middleware via Environment File

To change the middleware that is being used by the ROS2 software inside a docker container, edit the previously mentioned environment file.

Open the file with the following command:

```bash
sudo nano /etc/environment
```

In this file you will find the environment variable **RMW_IMPLEMENTATION**.

```bash
# ROS2 Middleware
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Since version 0.5.0, Cyclone DDS is assigned by default. To change the middleware, please select one from the following list and replace the current one.

* `rmw_fastrtps_cpp` (FastRTPS)
* `rmw_cyclonedds_cpp` (CycloneDDS)

The easiest way to make the change take effect is to reboot the robot.

```bash
sudo reboot
```

After the reboot each Docker container needs to be restarted. This must be done for every running EduArt container. In case of edu_robot on an IoT2050, please do following:

```bash
cd ~/edu_robot/docker/iot2050
docker compose down
docker compose up
``` 

## Setting the Middleware for a local ROS2 Environment
> **__Note:__** This is **not** necessary if changed the ROS2 middleware of docker containers to FastRTPS 

As already mentioned, every participant in the ROS2 network must use the same middleware due to compatibility concerns. This also applies to command line tools (ros2 topic list, ...). It is therefore necessary that the middleware is also switched to CycloneDDS on the system on which ROS2 is used. This is done using the same environment variable.

For a temporary change, the variable can be set via an `export` command in your terminal. However, this setting is only active for the current session!
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
----

If the middleware is to be changed permanently, it is advisable to add the `export` command in the `~/.bashrc` file. For this, open the file:
```bash
nano ~/.bashrc
```

Go to the bottom of the file and add the following line:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

To make the change take effect for existing terminal sessions, you need to source the file:
```bash
source ~/.bashrc
```