# Setup IoT2050

## SD Card Image

## Configure Devices on IoT2050

## Install Required Software

### Docker Engine

### Git

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
