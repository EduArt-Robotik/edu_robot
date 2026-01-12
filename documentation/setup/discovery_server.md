# Discovery Server Setup

ROS2 uses multicast UDP packets to discover topics. This is a convenient method when operating the robot in different networks. It eliminates the need to adjust IP addresses when changing networks. In some cases, however, UDP multicast causes problems or is simply not permitted. In these cases, a discovery server is a good option. However, this means losing flexibility and having to know the IP address of the discovery server.

For more background and details please visit this [page](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html).

## Setup Discovery Server

The server can be launched by following command:

```bash
fastdds discovery --server-id 0 --ip-address=192.168.0.101 --port=11811
```

If you are using an EduArt robot it can be enabled by setting the environment variable **ROS_DISCOVERY_SERVER**, for example in the file /etc/environment, to **ip_address:port**. In this case the eduard docker container will launch a discovery server. All other containers will listen to that discovery server.


