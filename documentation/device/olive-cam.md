# Olive Cam

The Olive Cam comes with a USB port that attaches itself as a network interface. ROS is already running on the camera itself, so no extra control software is required on the robot. However, if the ROS topics and services should be accessible outside the robot over the network, the network on the robot must be configured accordingly. 

This guide provides a quick explanation of how to set up the camera on the robot, how to connect to the camera and how to bridge the ROS over the interfaces.

## Connect the Camera

Connect the USB cable of the camera on a USB3 port on the robot. After a while (~1m) the camera is up and should be connected as network interface on the robot. To check if the device is working connect to the robot using SSH. Use

```bash
ip address
```

to check the available interfaces. On the IOT2050 device (IoT Bot) the print out will look like:

```bash
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eno2: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether 8c:f3:19:6c:87:50 brd ff:ff:ff:ff:ff:ff
    inet 192.168.1.103/24 brd 192.168.1.255 scope global dynamic noprefixroute eno2
       valid_lft 84600sec preferred_lft 84600sec
    inet6 fe80::68b5:11fc:5e2d:bc82/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
3: eno1: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc mq state DOWN group default qlen 1000
    link/ether 8c:f3:19:6c:87:4f brd ff:ff:ff:ff:ff:ff
4: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:d8:66:48:45 brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
5: enxa8f2fb1fa7d4: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UNKNOWN group default qlen 1000
    link/ether a8:f2:fb:1f:a7:d4 brd ff:ff:ff:ff:ff:ff
    inet 10.42.0.95/24 brd 10.42.0.255 scope global dynamic noprefixroute enxa8f2fb1fa7d4
       valid_lft 2089sec preferred_lft 2089sec
    inet6 fe80::490f:ed79:6c57:74b4/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
```

* **lo**: local loop
* **eno1**: first lan network
* **eno2**: second lan network (usually it is connected to the Wifi access point on the robot)
* **docker0**: docker network interface
* **enxa8f2fb1fa7d4**: the interface coming with the Olive camera.

As you can see the Olive cam uses the subnet 10.42.0.0/24. Execute following command to get the camera's ip address:

```bash
nmap -sn 10.42.0.0/24
```

> **Note**: if nmap is not install it can be installed using **sudo apt install nmap**.

nmap will display all available network devices connected to this subnet:

```bash
Nmap scan report for 10.42.0.7
Host is up (0.00093s latency).
Nmap scan report for 10.42.0.95
Host is up (0.00046s latency).
```

Here two are present. The first entry is the cam, the second entry the host (robot). To connect to the camera use following command:

```bash
ssh olive@10.42.0.7
```

The password is **one**. Maybe it is necessary to accept the SSH fingerprint (type in **yes** if asked). Now you are on the camera:

```bash
Linux robotics 5.10.10-rt24 #1 SMP PREEMPT Wed Oct 4 20:43:05 UTC 2023 armv7l

⠀⢀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⡀⠀⠀⣤⣤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⣿⣿⣿⠟⠋⠉⠉⠉⠛⢿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⣿⣿⡇⠀⠈⠿⠿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⣿⡟⠁⠀⠀⠀⠀⠀⠀⠀⠹⣿⡇⠀⠀⠀⠀⠀⠀⢀⣤⣶⣶⣶⣶⣄⡀⠀⠀⠀⣿⣿⡇⠀⣤⣤⣤⡄⠀⢤⣤⡄⠀⠀⠀⢀⣤⣤⠀⠀⢀⣤⣶⣶⣶⣦⣀⠀⠀
⠀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⡇⠀⠀⠀⠀⠀⢠⣿⣿⠛⠋⠙⠛⣿⣿⡄⠀⠀⣿⣿⡇⠀⠛⢻⣿⡇⠀⠘⣿⣿⡀⠀⠀⢸⣿⡏⠀⣰⣿⡿⠋⠉⠙⢻⣿⣆⠀ SOFTWARE
⠀⣿⠀⠀⠀⠀⢀⣀⠀⠀⠀⠀⣸⡇⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀⠀⠀⠸⣿⣿⠀⠀⣿⣿⡇⠀⠀⢸⣿⡇⠀⠀⢹⣿⣇⠀⢀⣿⣿⠁⠀⣿⣿⣥⣤⣤⣤⣬⣿⣿⠀ DEFINED
⠀⣿⣧⡀⠀⠀⣿⣿⡇⠀⠀⣠⣿⡇⠀⠀⠀⠀⠀⢿⣿⣆⠀⠀⠀⠀⢸⣿⣿⠀⠀⣿⣿⡇⠀⠀⢸⣿⡇⠀⠀⠀⣿⣿⡄⢸⣿⡏⠀⠀⣿⣿⡉⠉⠉⠉⢉⣉⣉⠀ ROBOT
⠀⣿⣿⣿⣦⣀⣈⣉⣀⣠⣾⣿⣿⡇⠀⠀⠀⠀⠀⠈⢿⣿⣶⣤⣤⣴⣿⡿⠃⠀⠀⣿⣿⡇⠀⠀⢸⣿⡇⠀⠀⠀⠸⣿⣧⣿⣿⠀⠀⠀⠘⢿⣿⣦⣤⣤⣾⣿⠏⠀ HARDWARE
⠀⠈⠙⠛⠛⠛⠛⠛⠛⠛⠛⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠛⠛⠉⠀⠀⠀⠀⠛⠛⠃⠀⠀⠘⠛⠃⠀⠀⠀⠀⠛⠛⠛⠃⠀⠀⠀⠀⠀⠙⠛⠛⠛⠋⠁⠀⠀
                               
                             www.olive-robotics.com
                             Copyrights © 2023-2024 

This device is powered by Olive Robotics GmbH - OLIX-OS version 2.2.1!
For more information please contact [support@olive-robotics.com].
```

