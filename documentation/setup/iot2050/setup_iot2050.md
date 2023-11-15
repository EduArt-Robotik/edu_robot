# First Time Setup of IoT2050

## SD Card Image

It is considered that the official ["Example Image V1.3.1"](https://support.industry.siemens.com/cs/document/109741799/downloads-f%C3%BCr-simatic-iot20x0?dti=0&lc=de-DE) provided by Siemens will be used for the IoT2050. If this is not the case it cloud lead in a misinterpretation of the game pad, because of an different kernel version.

### Flashing SD Card on Linux

### Flashing SD Card on Windows

## Configure Devices on IoT2050

## Install Required Software

### Docker Engine

### Git

### SSH

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