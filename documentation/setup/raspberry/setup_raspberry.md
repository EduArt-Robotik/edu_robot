# Setting up a Raspberry PI

## Enabling CAN Interfaces

1. Install Raspberry Pi OS or Ubuntu, Ubuntu 22.04.3 server (jammy jellyfish) has been tested
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

3. Add udev rules for CAN interfaces
The extension board for the Raspberry Pi provides three CANFD interfaces. To ensure that the naming of the interfaces is the same after each boot process, a udev rule must be created in the /etc/udev/rules.d directory. Create the file /etc/udev/rules.d/42-mcp251xfd.rules using nano:

```bash
sudo nano /etc/udev/rules.d/42-mcp251xfd.rules
```

and add the following content:

```console
KERNELS=="spi0.1", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="can0", TAG+="systemd", ENV{SYSTEMD_WANTS}="can0-attach.service"
KERNELS=="spi0.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="can1", TAG+="systemd", ENV{SYSTEMD_WANTS}="can1-attach.service"
KERNELS=="spi1.0", SUBSYSTEMS=="spi", DRIVERS=="mcp251xfd", ACTION=="add", NAME="can2", TAG+="systemd", ENV{SYSTEMD_WANTS}="can2-attach.service"
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
ExecStart=ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
```

After do the same for can1. Create the can1-attach by:

```bash
sudo nano /etc/systemd/system/can1-attach.service
```

And put following lines into it:

```bash
[Service]
Type=oneshot
ExecStart=ip link set can1 up type can bitrate 1000000 dbitrate 2000000 fd on
```

As last create the systemd service file for can2:

```bash
sudo nano /etc/systemd/system/can2-attach.service
```

And put following lines into it:

```bash
[Service]
Type=oneshot
ExecStart=ip link set can2 up type can bitrate 500000
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
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can 
4: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can 
5: can2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can 
```