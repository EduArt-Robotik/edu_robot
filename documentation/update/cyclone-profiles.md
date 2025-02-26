# Cyclone Profiles
When using Cyclone DDS as the ROS2 middleware, you need to tell cyclone which network interface to use.
If you don't Cyclone will decide itself which can lead to the situation that it maybe listens to your Wifi interface but in reality you use your RJ45-ethernet connection for communication with e.g. your robot.

This guide explains how to setup a profile on your local system and how to edit the profile of a docker container. In the last section there is a guide on how to check if ROS2 uses the right interface.

## Content
- [Cyclone Profiles](#cyclone-profiles)
  - [Content](#content)
  - [Create a Cyclone Profile for a local System](#create-a-cyclone-profile-for-a-local-system)
  - [Edit a Docker Cyclone Profile](#edit-a-docker-cyclone-profile)
  - [How to check your ROS2 Interfaces](#how-to-check-your-ros2-interfaces)

## Create a Cyclone Profile for a local System
You are probably using some kind of Linux system for your development or to simply connect to your robot. If this system also uses Cyclone DDS (which we recommend), you need to setup a profile due to the reasons mentioned above.

1. Check the names of available network interfaces:
```bash
ip a
```
Have a look for something like...
- `en..`(e.g. `enp1s0f0`) for ethernet connections
- `wl...` (e.g. `wlp2s0`) for wifi connections

2. Stop your ros2 daemon, if its already running<br>
```bash
ros2 daemon stop
```
If this doesn't return `The daemon is not running` or `The daemon has been stopped` you might need to log out and in again.

3. Create a folder on your local system<br>
Create a new folder for your profile with the following command:
```bash
mkdir ~/.ros/middlewareSettings/
```

>Note: The filepath and naming is only a recommendation. You can store/name the file wherever and whatever you like.

4. Create a new Cyclone profile <br>
Use this command to create and edit a new file:
```bash
nano ~/.ros/middlewareSettings/cyclone_profile_<interface_type>.xml
```
Since you probably have multiple available interfaces on your system (WIFI, ethernet,...) we recommend to create multiple Cyclone profiles for each of them. This makes it easier to choose between them at a later stage. To differenciate between the different profiles, append the type (e.g. `ethernet` or `wifi`) to the filename.<br> 

Paste the following content into your Cyclone profile:
```xml
<?xml version="1.0" encoding="utf-8"?>
<CycloneDDS
xmlns="https://cdds.io/config"
xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"
>
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="wlp2s0" priority="10" multicast="true" />
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
    </Domain>
</CycloneDDS>
```
Edit the `name` so that it contains the name of the interface you'd like to to use. Save (`Ctrl+S`) and close (`Ctrl+X`) the file. <br>

You should now have one or multiple Cyclone profiles.

5. Add the profile to your `.bashrc` <br>
You need to export a environment variable called `CYCLONEDDS_URI` to tell Cyclone which profile to use. For this, open your `.bashrc`:
```bash
nano ~/.bashrc
```

Add this to the bottom of the file:

```bash
#...
export CYCLONEDDS_URI=~/.ros/middlewareSettings/cyclone_profile_wifi.xml
``` 
Change the name of the profile that it matches yours. You can also prepare the export for different profiles and comment out all but one. This way you only need to edit a single line in your `.bashrc` to change the interface that Cyclone uses. This could look like this:
```bash
#...
# Wifi
export CYCLONEDDS_URI=~/.ros/.../cyclone_profile_wifi.xml
# Ethernet
# export CYCLONEDDS_URI=~/.ros/.../cyclone_profile_ethernet.xml
# ...
# export CYCLONEDDS_URI=~/.ros/...
```
Save (Ctrl + S) and close (Ctrl + C) the file.

For existing terminals you need to source the `.bashrc` again. Alternatively you can close the terminal and open a new one:
```bash
source ~/.bashrc
```

## Edit a Docker Cyclone Profile
Each EduArt docker comes with a `launch_content` folder that contains different files that get mounted and used by the container. It also contains a `cyclone_profile.xml` that specifies an interface. The default interface is configured in a way that it fits the Eduard robot hardware. If you use a container on your system and not on the robot you need to edit the Cyclone profile for this container.

The following explanations uses the [edu_nodered_ros2_plugin](https://github.com/EduArt-Robotik/edu_nodered_ros2_plugin) as an example since this is the most likely container you'll use on your own system.

1. Navigate to the Cyclone profile <br>
```bash
cd <path_to_nodered_repo>/docker/<platform>/launch_content
# example:
cd ~/repos/edu_nodered_ros2_plugin/docker/raspberry/launch_content
```

2. Modify the existing profile
```bash
nano cyclone_profile.xml
``` 
Enter the name of the interface you want to use. See the [previous guide](#create-a-cyclone-profile-for-a-local-system) on how to do that. Afterwards, save and exit the file.

3. Restart your container
If the container was already running you need to restart it fully. For this, we navigate on folder upwards to the `docker-compose.yaml`
```bash
cd ..
docker compose down
docker compose up
```

## How to check your ROS2 Interfaces

> **Note:** Work in progress

```bash
netstat -gn
```
Search for multicast IP `239.255.0.1`

```bash
sudo netstat -tulpn
```