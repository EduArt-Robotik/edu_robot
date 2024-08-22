# Setting up a Raspberry Pi

## Installing Ubuntu on Raspberry Pi

We currently recommend installing Ubuntu 24.04 LTS on the Raspberry. There is a ROS2 installation for this distribution, which is available as a binary.

Please visit this [link](https://ubuntu.com/download/raspberry-pi) for installation instructions. We recommend that you only install the server version. However, it is up to you to decide exactly which version to install. You are also free to install the desktop version.

### Flashing SD Card 

Please use the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to install your favorite OS for Raspberry Pi. We recommend to install Ubuntu 24.04 LTS, because there exists a native ROS2 installation for it.

We also recommend to enable SSH via the tool and create following user:
* username : user
* password : eduart

But please feel free to configure it like you want.

The image is now flashed on the SD card. Please insert it into the Raspberry Pi and boot it up.

## Enabling CAN Interfaces

> **Note**: sudo privileges are required.

1. Login into the raspberry.
2. Update and install packages
```console
sudo apt update
sudo apt upgrade
sudo apt install can-utils build-essential git
```

3. Enable CAN hardware

Open the file "config.txt" using nano

```bash
sudo nano /boot/firmware/config.txt
```

And add following lines to the file:

```bash
dtoverlay=spi1-2cs
dtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25
dtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=13
dtoverlay=mcp251xfd,spi1-0,oscillator=40000000,interrupt=24
```

3. Add udev rules for CAN interfaces </br>
The extension board for the Raspberry Pi provides three CANFD interfaces. To ensure that the naming of the interfaces is the same after each boot process, a udev rule must be created in the /etc/udev/rules.d directory. Create the file /etc/udev/rules.d/42-mcp251xfd.rules using nano:

```bash
sudo nano /etc/udev/rules.d/42-mcp251xfd.rules
```

and add the following content:

```console
KERNELS=="spi0.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="eduart-can0", TAG+="systemd", ENV{SYSTEMD_WANTS}="can0-attach.service"
KERNELS=="spi0.1", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="eduart-can1", TAG+="systemd", ENV{SYSTEMD_WANTS}="can1-attach.service"
KERNELS=="spi1.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="eduart-can2", TAG+="systemd", ENV{SYSTEMD_WANTS}="can2-attach.service"
```

In this way, the CAN interface for the motor controller is always named can2. The can0 and can1 interfaces can be accessed via the sockets on the expansion board (see labeling on the board) and are intended for connecting the flexible sensor ring from EduArt.

As next step a systemd service should be defined to bring up all CAN interfaces at boot up. Create the file can0-attach.service by:

```bash
sudo nano /etc/systemd/system/can0-attach.service
```

and add following lines to it:

```bash
[Service]
Type=oneshot
ExecStart=ip link set eduart-can0 up type can bitrate 1000000 dbitrate 2000000 fd on
```

After do the same for can1. Create the can1-attach by:

```bash
sudo nano /etc/systemd/system/can1-attach.service
```

And put following lines into it:

```bash
[Service]
Type=oneshot
ExecStart=ip link set eduart-can1 up type can bitrate 1000000 dbitrate 2000000 fd on
```

As last create the systemd service file for can2:

```bash
sudo nano /etc/systemd/system/can2-attach.service
```

And put following lines into it:

```bash
[Service]
Type=oneshot
ExecStart=ip link set eduart-can2 up type can bitrate 500000
```

Now reboot the Raspberry Pi by following command:

```bash
sudo reboot
```

After the reboot you can check if all CAN interface are working and up by:

```bash
ip address
```

All three CAN interface should be listed and in state UP:

```bash
3: eduart-can1: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP group default qlen 10
    link/can 
4: eduart-can0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP group default qlen 10
    link/can 
5: eduart-can2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can
```

## Install ROS Control Software

Please follow these steps to prepare the installation of the ROS2 control software:

1. [Install Docker Engine](../iot2050/setup_iot2050.md#docker-engine)
2. [Prepare Environment](../iot2050/setup_iot2050.md#prepare-environment)

### Get Control Software and Launch it

First clone the Git repository by executing the command:

```bash
cd ~
git clone --branch main https://github.com/EduArt-Robotik/edu_robot.git
cd ~/edu_robot/docker/raspberry
```

In this folder a docker compose file is located. It is used to launch the basic control software including joy node and a joy interpreter. Launch the software by:

```bash
docker compose up
```

The software will be registered for auto start after the robot boots up. If you want to remove it from the autostart execute following command inside the same folder:

```bash
docker compose down
```

### Native ROS2 Installation

The ROS2 software for controlling the robot's hardware runs in Docker. Other applications can also be installed via Docker. It is therefore not necessary to install ROS2 natively on the system.

However, for all those who prefer to have ROS2 installed natively, this can be done with the following steps:

* Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

* Enable Required Repositories

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

* Install Development Tools

```bash
sudo apt update && sudo apt upgrade && sudo apt install ros-dev-tools
```

* Install ROS2

```bash
sudo apt install ros-jazzy-ros-base
```

* Source ROS into your .bashrc
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
* 