## Deploying on IPC127e

> **Note**: the documentation is rewritten at the moment. A update will be available soon!

### Use Prebuilt Docker Images

The easiest way, and one that is usually quite sufficient, is to use a prebuilt Docker image. All released versions of edu_robot software are usually available. 

We provide a docker compose file for the IPC127e. Either [download the file](docker/ipc127e/docker-compose.yaml) on IPC127e or clone the repository and navigate to the file:

```bash
git clone https://github.com/EduArt-Robotik/edu_robot.git
cd edu_robot_control/docker/ipc127e
```

Then execute following command inside the folder where the ["docker-compose.yaml"](docker/ipc127e/docker-compose.yaml) is located:

```bash
docker compose up
```

Inside the docker compose file a namespace is defined. This namespace can freely be modified. We recommend to reflect the robot color with this namespace. But in general do it like you want.

For removing the docker container execute the command:

```bash
docker compose down
```

at the location of the docker compose file.