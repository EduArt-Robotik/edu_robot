# First Time Setup of IoT2050

## SD Card Image

It is considered that the official ["Example Image V1.3.1"](https://support.industry.siemens.com/cs/document/109741799/downloads-f%C3%BCr-simatic-iot20x0?dti=0&lc=de-DE) provided by Siemens will be used for the IoT2050. If this is not the case it cloud lead in a misinterpretation of the game pad, because of an different kernel version.

### Flashing SD Card on Linux

### Flashing SD Card on Windows

## Configure Devices on IoT2050

## Install Required Software

### Docker Engine

Docker engine is used for executing our ROS software on the robot. Therefor the engine has to be installed. Please follow these [instructions](https://docs.docker.com/engine/install/debian/) including the [post install instructions](https://docs.docker.com/engine/install/linux-postinstall/).

> **Note:** We have also summarized the instructions here, but it could happen that they are not up to date:

1. Set up Docker's apt repository.

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

2. Install the Docker packages.

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

3. Add your user to the docker group.

```bash
sudo usermod -aG docker $USER
```

4. Run the following command to activate the changes to groups:

```bash
newgrp docker
```

5. Verify that you can run docker commands without sudo:

```bash
docker run hello-world
```

### Git

Install git by following command:

```bash
sudo apt update
sudo apt install git
```

### SSH

Install ssh by following command:

```bash
sudo apt update
sudo apt install openssh-server
```

## Install ROS Control Software

### Prepare Environment

By default the used namespace for ROS topics, services and tf is 'eduard'. If multiple robots are connected to the same network, each robot must be given its own namespace in order to differentiate between the robots. In order to apply this namespace to all ROS nodes (control software) a environment variable needs to be set. The easiest way is defining it in the file '/etc/environment' by:

```bash
sudo nano /etc/environment
```

Now defines the variable. Here in this example 'eduard/blue' was chosen. This namespace will respected by all EduArt ROS nodes.

```bash
# EduArt
EDU_ROBOT_NAMESPACE=eduard/blue
```

### Get Software and Launch it

First clone the Git repository by executing the command:

```bash
cd ~
git clone --branch main https://github.com/EduArt-Robotik/edu_robot.git
cd ~/edu_robot/docker/iot2050
```

In this folder a docker compose file is located. It is used to launch the basic control software including joy node and a joy interpreter. Launch the software by:

```bash
docker compose up
```

The software will be registered for auto start after the robot boots up. If you want to remove it from the autostart execute following command inside the same folder:

```bash
docker compose down
```

#### Modifying Parameter of Control Software

If you want to change the default parameter go to the folder 'launch_content' by:

```bash
cd ~/edu_robot/docker/iot2050/launch_content
```

Inside this folder two launch files and two parameter file are located. Usually it shouldn't be necessary to modify the launch files. So please let it untouched. The first parameter file ([eduard-iot2050.yaml](../../../docker/iot2050/launch_content/eduard-iot2050.yaml)) defines all parameter used to robot control software (edu_robot). Here for example the kinematic could be changed or the collision avoidance could be adapted.

The second parameter file ([remote_control.yaml](../../../docker/iot2050/launch_content/remote_control.yaml)) defines the user input interpretation. Here for example the joy stick button assignment could be changed or the maximum velocity at full throttle could be adapted.

To apply the new set parameter the software has to be relaunched. This can be done by:

```bash
cd ~/edu_robot/docker/iot2050
docker compose down
docker compose up
```